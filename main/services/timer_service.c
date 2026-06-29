/**
 * @file timer_service.c
 * @brief 定时任务管理服务实现
 *
 * 【已实现功能】
 *   1. SNTP 时间同步（WiFi 连接后自动同步，3 个 NTP 服务器）
 *   2. 定时任务调度（灯光/空调按周计划开关，NVS 持久化）
 *
 * 【定时任务调度逻辑】
 *   - 上电时从 NVS 加载任务列表（最多 APP_CFG_TIMER_MAX_TASKS 条）
 *   - SNTP 同步成功后进入主循环，每 30 秒检查一次
 *   - 匹配条件：星期掩码 & 小时 & 分钟，且该分钟内只触发一次
 *   - 仅控制手动设备（灯光 CH13、空调 CH14），不干预 FSM 管辖的 CH0-CH12
 */
#include "timer_service.h"
#include "common/app_config.h"
#include "common/app_events.h"
#include "common/data_store.h"
#include "services/actuator_service.h"
#include "services/cabin_fsm.h"         /* CH_LIGHT, CH_AC */

#include "esp_log.h"
#include "esp_sntp.h"
#include <time.h>
#include <sys/time.h>
#include <string.h>

static const char *TAG = "timer_svc";

/* ════════════════════════════════════════════════
 * 定时任务数据结构
 * ════════════════════════════════════════════════ */

typedef struct {
    uint8_t  hour;       /* 触发小时 (0-23) */
    uint8_t  minute;     /* 触发分钟 (0-59) */
    uint8_t  channel;    /* 执行器通道（CH_LIGHT=13 或 CH_AC=14） */
    bool     action;     /* true=开, false=关 */
    bool     enabled;    /* 任务是否启用 */
    uint8_t  weekdays;   /* 星期掩码 bit0=周日...bit6=周六, 0xFF=每天 */
} timer_task_entry_t;

static timer_task_entry_t s_task_list[APP_CFG_TIMER_MAX_TASKS];
static int s_task_count = 0;

/* ── SNTP 同步状态（回调设置，比轮询 get_sync_status 更可靠）── */
static volatile bool s_time_synced = false;

/* ════════════════════════════════════════════════
 * SNTP 时间同步
 * ════════════════════════════════════════════════ */

static void sntp_sync_notification(struct timeval *tv)
{
    s_time_synced = true;

    time_t now;
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "SNTP 同步成功: %04d-%02d-%02d %02d:%02d:%02d (CST)",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

static void sntp_start(void)
{
    ESP_LOGI(TAG, "正在初始化 SNTP...");

    /* 设置时区（中国 UTC+8） */
    setenv("TZ", APP_CFG_TIMEZONE, 1);
    tzset();

    /* 配置 SNTP（仅配置，不 init） */
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, APP_CFG_SNTP_SERVER);         /* 阿里云 NTP */
    esp_sntp_setservername(1, APP_CFG_SNTP_SERVER_BACKUP);  /* 腾讯云 NTP 备用 */
    esp_sntp_setservername(2, "pool.ntp.org");             /* 国际 NTP 备用 */
    sntp_set_time_sync_notification_cb(sntp_sync_notification);

    /* 启动 SNTP */
    esp_sntp_init();

    /* 等待同步完成（最多 10 秒），100ms 轮询回调 flag */
    TickType_t start = xTaskGetTickCount();
    const uint32_t timeout_ms = 10000;

    while (!s_time_synced) {
        vTaskDelay(pdMS_TO_TICKS(100));

        uint32_t elapsed = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;
        if (elapsed >= timeout_ms) {
            ESP_LOGW(TAG, "SNTP 同步超时（%lu ms），将继续运行", (unsigned long)elapsed);
            esp_sntp_stop();
            return;
        }
    }
}

/* ════════════════════════════════════════════════
 * NVS 持久化
 * ════════════════════════════════════════════════ */

/** 从 NVS 加载定时任务列表 */
static void load_tasks_from_nvs(void)
{
    size_t len = sizeof(s_task_list);
    esp_err_t ret = data_store_get_blob("timer_tasks", s_task_list, &len);

    if (ret == ESP_OK && len > 0) {
        /* 计算有效条目数（按结构体大小整除） */
        s_task_count = (int)(len / sizeof(timer_task_entry_t));
        if (s_task_count > APP_CFG_TIMER_MAX_TASKS) {
            s_task_count = APP_CFG_TIMER_MAX_TASKS;
        }
        ESP_LOGI(TAG, "从 NVS 加载 %d 条定时任务", s_task_count);
    } else {
        s_task_count = 0;
        ESP_LOGI(TAG, "NVS 无定时任务数据，使用空列表");
    }
}

/** 保存定时任务列表到 NVS（供 MQTT/串口屏添加任务时调用） */
static esp_err_t __attribute__((unused)) save_tasks_to_nvs(void)
{
    if (s_task_count <= 0) {
        return ESP_OK;  /* 空列表不写入 */
    }
    size_t len = (size_t)s_task_count * sizeof(timer_task_entry_t);
    return data_store_set_blob("timer_tasks", s_task_list, len);
}

/* ════════════════════════════════════════════════
 * 定时任务调度主循环
 * ════════════════════════════════════════════════ */

static void timer_task(void *arg)
{
    ESP_LOGI(TAG, "timer_task 已启动，等待 WiFi 连接...");

    /* 等待 WiFi 连接成功 */
    xEventGroupWaitBits(g_system_events, EVT_WIFI_CONNECTED,
                        pdFALSE, pdFALSE, portMAX_DELAY);

    ESP_LOGI(TAG, "WiFi 已连接，开始 SNTP 同步");

    /* 启动 SNTP 时间同步 */
    sntp_start();

    if (!s_time_synced) {
        ESP_LOGW(TAG, "SNTP 未同步，定时任务可能不准确");
    }

    if (s_task_count == 0) {
        ESP_LOGI(TAG, "无定时任务，调度循环空转（等待 MQTT/串口屏添加任务）");
    }

    /* ── 调度主循环：每 30 秒检查一次 ── */
    int8_t last_triggered_min = -1;     /* 上次触发的分钟（防止同一分钟重复触发） */

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));   /* 30 秒检查一次 */

        if (s_task_count == 0) {
            continue;   /* 无任务，跳过 */
        }

        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);

        int cur_min = timeinfo.tm_min;

        /* 同一分钟内只触发一次（30秒轮询可能在同一分钟内检查两次） */
        if (cur_min == last_triggered_min) {
            continue;
        }

        uint8_t wday_mask = (uint8_t)(1 << timeinfo.tm_wday);  /* bit0=周日 */

        for (int i = 0; i < s_task_count; i++) {
            timer_task_entry_t *t = &s_task_list[i];

            if (!t->enabled) continue;
            if (t->hour   != (uint8_t)timeinfo.tm_hour) continue;
            if (t->minute != (uint8_t)timeinfo.tm_min)  continue;
            if (!(t->weekdays & wday_mask)) continue;

            /* 安全检查：只允许控制手动设备 */
            if (t->channel != CH_LIGHT && t->channel != CH_AC) {
                ESP_LOGW(TAG, "定时任务[%d] 通道 %d 不是手动设备，跳过", i, t->channel);
                continue;
            }

            actuator_set(t->channel, t->action);
            ESP_LOGI(TAG, "定时任务触发: [%d] %02d:%02d 周%d CH%d=%s",
                     i, t->hour, t->minute, timeinfo.tm_wday,
                     t->channel, t->action ? "开" : "关");
        }

        last_triggered_min = (int8_t)cur_min;
    }
}

/* ════════════════════════════════════════════════
 * 公共 API
 * ════════════════════════════════════════════════ */

bool timer_service_is_time_synced(void)
{
    return s_time_synced;
}

esp_err_t timer_service_init(void)
{
    memset(s_task_list, 0, sizeof(s_task_list));
    s_task_count = 0;

    /* 从 NVS 加载已保存的定时任务 */
    load_tasks_from_nvs();

    /* 打印已加载的任务 */
    for (int i = 0; i < s_task_count; i++) {
        timer_task_entry_t *t = &s_task_list[i];
        ESP_LOGI(TAG, "  任务[%d]: %02d:%02d CH%d=%s 星期=0x%02X %s",
                 i, t->hour, t->minute, t->channel,
                 t->action ? "开" : "关", t->weekdays,
                 t->enabled ? "启用" : "禁用");
    }

    return ESP_OK;
}

esp_err_t timer_service_start(void)
{
    BaseType_t ret = xTaskCreate(timer_task, "timer_task", 4096, NULL, 3, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 timer_task 失败");
        return ESP_FAIL;
    }
    return ESP_OK;
}
