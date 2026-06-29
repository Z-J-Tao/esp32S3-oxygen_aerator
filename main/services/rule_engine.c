/**
 * @file rule_engine.c
 * @brief 规则引擎实现 — 安全监测、报警分发、故障保护（论文 3.1.3、5.3 节）
 *
 * 【报警总览】
 *   温度（两级）      ：> 40℃ 警告 → > 50℃ 故障（仅报警+MQTT，50℃触发FSM_EVT_FAULT）
 *   湿度（高低两端）  ：< 30%RH 低湿 → > 80%RH 高湿（仅报警+MQTT，不停机）
 *   氧浓度（三级高氧）：≥22% 换气 → ≥25% 加大换气+声光 → ≥30% 紧急泄压+安全模式
 *   压力（三级软件）  ：≥58KPa 停止增压 → ≥60KPa 泄压+异常保护 → ≥62KPa 排气全开+紧急模式
 *   压力变化率       ：>15KPa/min → 停机保护
 *   传感器离线       ：连续3次无响应 → 停机保护
 *
 * 【防抖策略】
 *   回差法（hysteresis）：报警触发后设置 active 标志，
 *   需回落到阈值以下一定幅度才解除。
 *
 * 【严重异常统一处理流程】
 *   1. 关闭空气泵、增氧机等全部执行器
 *   2. 打开排气阀，启动泄压
 *   3. 声光报警 + 语音提醒 + MQTT 推送
 *   4. 记录故障信息(NVS)
 *   5. FSM 进入安全模式，禁止自动重启
 */
#include "rule_engine.h"
#include "common/app_config.h"
#include "common/app_events.h"
#include "common/data_store.h"
#include "services/sensor_service.h"
#include "services/cabin_fsm.h"
#include "services/actuator_service.h"
#include "services/voice_service.h"
#include "network/app_mqtt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static const char *TAG = "rule_eng";

/* ════════════════════════════════════════════════
 * 报警规则结构体（可通过 NVS 持久化修改）
 * ════════════════════════════════════════════════ */

typedef struct {
    /* 温度（两级，仅报警不停机） */
    float temp_warn;            /* 默认 APP_CFG_ALARM_TEMP_WARN       (40.0) */
    float temp_fault;           /* 默认 APP_CFG_ALARM_TEMP_FAULT      (50.0) */
    /* 湿度（高低两端，仅报警不停机） */
    float humidity_low;         /* 默认 APP_CFG_ALARM_HUMIDITY_LOW    (30.0) */
    float humidity_high;        /* 默认 APP_CFG_ALARM_HUMIDITY_HIGH   (80.0) */
    /* 氧浓度（三级，高氧方向） */
    float oxygen_level1;        /* 默认 APP_CFG_ALARM_O2_LEVEL1       (22.0) */
    float oxygen_level2;        /* 默认 APP_CFG_ALARM_O2_LEVEL2       (25.0) */
    float oxygen_level3;        /* 默认 APP_CFG_ALARM_O2_LEVEL3       (30.0) */
    /* 压力（三级软件） */
    float pressure_warn;        /* 默认 APP_CFG_ALARM_PRESSURE_WARN   (58.0) */
    float pressure_fault;       /* 默认 APP_CFG_ALARM_PRESSURE_FAULT  (60.0) */
    float pressure_emerg;       /* 默认 APP_CFG_ALARM_PRESSURE_EMERG  (62.0) */
    /* 压力变化率 */
    float pressure_rate_max;    /* 默认 APP_CFG_ALARM_PRESSURE_RATE   (15.0) KPa/min */
    /* 传感器离线 */
    uint8_t sensor_timeout;     /* 默认 APP_CFG_ALARM_SENSOR_TIMEOUT  (3) */
} alarm_rules_t;

static alarm_rules_t s_rules;

/* ════════════════════════════════════════════════
 * 报警状态（防抖用 — 回差法）
 * ════════════════════════════════════════════════ */

static struct {
    bool temp_warn_active;
    bool temp_fault_active;
    bool humidity_low_active;
    bool humidity_high_active;
    bool o2_level1_active;
    bool o2_level2_active;
    bool o2_level3_active;
    bool pressure_warn_active;
    bool pressure_fault_active;
    bool pressure_emerg_active;
    bool pressure_rate_active;
    uint8_t sensor_fail_count;      /* 连续超时/无效计数 */
    float   prev_pressure_kpa;      /* 上一次压力值（用于计算变化率） */
    int64_t prev_pressure_time_us;  /* 上一次压力采样时间 (esp_timer 微秒) */
} s_alarm = {0};

/* ── 回差值（防止报警边界反复触发）── */
#define HYSTERESIS_TEMP     2.0f    /* 温度回差 2℃ */
#define HYSTERESIS_HUMIDITY 5.0f    /* 湿度回差 5%RH */
#define HYSTERESIS_O2       1.0f    /* 氧浓度回差 1% */
#define HYSTERESIS_PRESSURE 2.0f    /* 压力回差 2KPa */

/* ════════════════════════════════════════════════
 * 报警历史环形缓冲 (供串口屏 Page 3 + 未来 MQTT 历史查询)
 *
 * 写: 触发新报警时 record_alarm() → 互斥锁保护下追加, 满则覆盖最旧
 * 读: rule_engine_get_recent() → 互斥锁保护下按"最新→最旧"复制到调用者数组
 * ════════════════════════════════════════════════ */

static struct {
    alarm_record_t  ring[ALARM_HISTORY_CAPACITY];
    uint8_t         write_idx;      /* 下一个写入位置, 0 ~ CAPACITY-1 */
    uint8_t         count;          /* 当前有效条数, 上限为 CAPACITY */
    SemaphoreHandle_t mutex;
} s_alarm_log = {0};

/**
 * @brief 把一条报警写入环形缓冲 + 透传到 MQTT (统一出口)
 *
 * 同时执行三件事, 避免在每个报警分支里重复写三遍:
 *   1. 推入历史环形缓冲 (供屏幕报警页查询)
 *   2. 调用 mqtt_publish_alarm() 推送给云端
 *   3. 不做日志/语音 — 那两件由调用方控制 (不同报警的语音 ID 不同)
 *
 * 设计权衡: 不在 record_alarm 内部 snprintf JSON, 是因为每种报警的 JSON
 * 结构略有差异 (比如压力变化率没有 threshold, 而是 rate); 让调用方自己拼好
 * 字符串传入更灵活, 也避免在锁内做长字符串格式化。
 *
 * @param level     等级 (WARN / FAULT)
 * @param type      类型字符串 (与 MQTT JSON 中的 "type" 字段保持一致)
 * @param value     实际值
 * @param threshold 阈值 (无意义时传 0)
 * @param mqtt_json 完整的 MQTT alert JSON (NULL 则不推送)
 */
static void record_alarm(alarm_level_t level, const char *type,
                         float value, float threshold,
                         const char *mqtt_json)
{
    /* === 1. 推入环形缓冲 === */
    if (s_alarm_log.mutex != NULL &&
        xSemaphoreTake(s_alarm_log.mutex, pdMS_TO_TICKS(50)) == pdTRUE) {

        alarm_record_t *slot = &s_alarm_log.ring[s_alarm_log.write_idx];
        slot->timestamp = time(NULL);   /* SNTP 未同步时 = 时间戳很小, UI 自己识别处理 */
        slot->level     = level;
        slot->value     = value;
        slot->threshold = threshold;
        strncpy(slot->type, type ? type : "?", sizeof(slot->type) - 1);
        slot->type[sizeof(slot->type) - 1] = '\0';

        s_alarm_log.write_idx = (s_alarm_log.write_idx + 1) % ALARM_HISTORY_CAPACITY;
        if (s_alarm_log.count < ALARM_HISTORY_CAPACITY) {
            s_alarm_log.count++;
        }

        xSemaphoreGive(s_alarm_log.mutex);
    }

    /* === 2. 推送 MQTT (锁外执行, 避免阻塞历史读取) === */
    if (mqtt_json != NULL) {
        mqtt_publish_alarm(APP_CFG_MQTT_ALERT_TOPIC, mqtt_json);
    }
}

/* ════════════════════════════════════════════════
 * 严重异常统一处理函数
 * ════════════════════════════════════════════════ */

/**
 * @brief 严重异常统一处理
 *
 * 执行流程：
 *   1. 关闭增氧机、增氧泵、全部进气阀
 *   2. 打开大流量排气阀 + 排气风扇
 *   3. 语音播报 + MQTT 告警推送
 *   4. 向 FSM 发送 FAULT 事件 → 安全模式
 *
 * @param reason  故障原因描述（用于日志和 MQTT）
 * @param alert_json  MQTT 告警 JSON 字符串（可为 NULL 则用 reason 构造）
 */
static void emergency_shutdown(const char *reason, const char *alert_json)
{
    ESP_LOGE(TAG, "严重异常: %s → 执行紧急停机", reason);

    /* 1. 关闭所有供气设备 */
    actuator_set(CH_OXYGEN_GEN,  false);    /* 关制氧机 */
    actuator_set(CH_OXYGEN_PUMP, false);    /* 关增氧泵 */
    actuator_set(CH_INTAKE_LOW,  false);    /* 关进气阀（全部） */
    actuator_set(CH_INTAKE_MED,  false);
    actuator_set(CH_INTAKE_FAST, false);

    /* 2. 打开排气阀泄压 */
    actuator_set(CH_DEFLATE_HIGH_O, true);  /* 大流量排气阀-开 */
    actuator_set(CH_DEFLATE_HIGH_C, false); /* 大流量排气阀-关闭端断电 */
    actuator_set(CH_EXHAUST_FAN,    true);  /* 排气风扇 */

    /* 3. 声光报警 + 语音 + MQTT */
    voice_service_play(VOICE_EMERGENCY);

    /* 注: emergency_shutdown 自身不进 record_alarm — 由调用方在更早的位置
     * (具体的报警判定处) 写入历史, 那里有完整的 type/value/threshold;
     * 这里只负责 MQTT 透传 (兜底, 万一上层没传 alert_json) 和触发 FAULT 事件 */
    if (alert_json != NULL) {
        mqtt_publish_alarm(APP_CFG_MQTT_ALERT_TOPIC, alert_json);
    } else {
        /* 构造简单的告警 JSON */
        char buf[192];
        snprintf(buf, sizeof(buf), "{\"type\":\"emergency\",\"reason\":\"%s\"}", reason);
        mqtt_publish_alarm(APP_CFG_MQTT_ALERT_TOPIC, buf);
    }

    /* 4. FSM 进入安全模式，禁止自动重启 */
    cabin_fsm_send_event(FSM_EVT_FAULT);
}

/* ════════════════════════════════════════════════
 * 各类报警检测函数
 * ════════════════════════════════════════════════ */

/**
 * @brief 温度检查（两级，50℃触发 FAULT，40℃仅报警）
 */
static void check_temperature(const sensor_data_t *data)
{
    /* ── 二级：温度故障 (> 50℃) → FSM_EVT_FAULT ── */
    if (data->temperature > s_rules.temp_fault) {
        if (!s_alarm.temp_fault_active) {
            s_alarm.temp_fault_active = true;
            ESP_LOGE(TAG, "温度故障: %.1f℃ > %.1f℃", data->temperature, s_rules.temp_fault);

            voice_service_play(VOICE_ALARM_TEMP);

            char json[128];
            snprintf(json, sizeof(json),
                     "{\"type\":\"temp_fault\",\"value\":%.1f,\"threshold\":%.1f}",
                     data->temperature, s_rules.temp_fault);
            record_alarm(ALARM_LEVEL_FAULT, "temp_fault",
                         data->temperature, s_rules.temp_fault, json);

            cabin_fsm_send_event(FSM_EVT_FAULT);
        }
    } else if (data->temperature < s_rules.temp_fault - HYSTERESIS_TEMP) {
        s_alarm.temp_fault_active = false;
    }

    /* ── 一级：温度警告 (> 40℃) → 仅报警 ── */
    if (data->temperature > s_rules.temp_warn && !s_alarm.temp_fault_active) {
        if (!s_alarm.temp_warn_active) {
            s_alarm.temp_warn_active = true;
            ESP_LOGW(TAG, "温度警告: %.1f℃ > %.1f℃", data->temperature, s_rules.temp_warn);

            voice_service_play(VOICE_ALARM_TEMP);

            char json[128];
            snprintf(json, sizeof(json),
                     "{\"type\":\"temp_warn\",\"value\":%.1f,\"threshold\":%.1f}",
                     data->temperature, s_rules.temp_warn);
            record_alarm(ALARM_LEVEL_WARN, "temp_warn",
                         data->temperature, s_rules.temp_warn, json);
        }
    } else if (data->temperature < s_rules.temp_warn - HYSTERESIS_TEMP) {
        s_alarm.temp_warn_active = false;
    }
}

/**
 * @brief 湿度检查（高低两端，仅报警不停机）
 */
static void check_humidity(const sensor_data_t *data)
{
    /* ── 高湿警告 (> 80%RH) ── */
    if (data->humidity > s_rules.humidity_high) {
        if (!s_alarm.humidity_high_active) {
            s_alarm.humidity_high_active = true;
            ESP_LOGW(TAG, "湿度偏高: %.1f%%RH > %.1f%%RH", data->humidity, s_rules.humidity_high);

            voice_service_play(VOICE_ALARM_TEMP);   /* 复用温度报警语音 */

            char json[128];
            snprintf(json, sizeof(json),
                     "{\"type\":\"humidity_high\",\"value\":%.1f,\"threshold\":%.1f}",
                     data->humidity, s_rules.humidity_high);
            record_alarm(ALARM_LEVEL_WARN, "humidity_high",
                         data->humidity, s_rules.humidity_high, json);
        }
    } else if (data->humidity < s_rules.humidity_high - HYSTERESIS_HUMIDITY) {
        s_alarm.humidity_high_active = false;
    }

    /* ── 低湿警告 (< 30%RH) ── */
    if (data->humidity < s_rules.humidity_low) {
        if (!s_alarm.humidity_low_active) {
            s_alarm.humidity_low_active = true;
            ESP_LOGW(TAG, "湿度偏低: %.1f%%RH < %.1f%%RH", data->humidity, s_rules.humidity_low);

            voice_service_play(VOICE_ALARM_TEMP);   /* 复用温度报警语音 */

            char json[128];
            snprintf(json, sizeof(json),
                     "{\"type\":\"humidity_low\",\"value\":%.1f,\"threshold\":%.1f}",
                     data->humidity, s_rules.humidity_low);
            record_alarm(ALARM_LEVEL_WARN, "humidity_low",
                         data->humidity, s_rules.humidity_low, json);
        }
    } else if (data->humidity > s_rules.humidity_low + HYSTERESIS_HUMIDITY) {
        s_alarm.humidity_low_active = false;
    }
}

/**
 * @brief 氧浓度检查（三级，高氧方向，从高到低判断避免低级覆盖高级）
 */
static void check_oxygen(const sensor_data_t *data)
{
    /* ── 三级故障 (O2 >= 30%): 紧急泄压 + 出舱 + 安全模式 ── */
    if (data->oxygen_percent >= s_rules.oxygen_level3) {
        if (!s_alarm.o2_level3_active) {
            s_alarm.o2_level3_active = true;
            ESP_LOGE(TAG, "氧浓度三级故障: %.1f%% >= %.1f%%",
                     data->oxygen_percent, s_rules.oxygen_level3);

            /* 关闭全部供氧设备 */
            actuator_set(CH_OXYGEN_GEN,  false);
            actuator_set(CH_OXYGEN_PUMP, false);

            /* 紧急泄压：排气阀全开 */
            actuator_set(CH_DEFLATE_HIGH_O, true);
            actuator_set(CH_DEFLATE_HIGH_C, false);
            actuator_set(CH_EXHAUST_FAN,    true);

            char json[128];
            snprintf(json, sizeof(json),
                     "{\"type\":\"oxygen_high_3\",\"value\":%.1f,\"threshold\":%.1f}",
                     data->oxygen_percent, s_rules.oxygen_level3);
            /* 先记历史 (NULL 表示不在此处发 MQTT, 由下方 emergency_shutdown 统一发) */
            record_alarm(ALARM_LEVEL_FAULT, "oxygen_high_3",
                         data->oxygen_percent, s_rules.oxygen_level3, NULL);
            emergency_shutdown("氧浓度三级故障(>=30%)-立即出舱", json);
        }
    } else if (data->oxygen_percent < s_rules.oxygen_level3 - HYSTERESIS_O2) {
        s_alarm.o2_level3_active = false;
    }

    /* ── 二级报警 (O2 >= 25%): 加大换气 + 声光报警 ── */
    if (data->oxygen_percent >= s_rules.oxygen_level2 && !s_alarm.o2_level3_active) {
        if (!s_alarm.o2_level2_active) {
            s_alarm.o2_level2_active = true;
            ESP_LOGW(TAG, "氧浓度二级报警: %.1f%% >= %.1f%%",
                     data->oxygen_percent, s_rules.oxygen_level2);

            /* 关增氧，全开进/排气阀加大换气 */
            actuator_set(CH_OXYGEN_GEN,     false);
            actuator_set(CH_OXYGEN_PUMP,    false);
            actuator_set(CH_INTAKE_FAST,    true);      /* 快速进气 */
            actuator_set(CH_DEFLATE_NORM_O, true);      /* 正常泄气阀开 */
            actuator_set(CH_DEFLATE_NORM_C, false);
            actuator_set(CH_EXHAUST_FAN,    true);      /* 排气风扇 */

            voice_service_play(VOICE_ALARM_OXYGEN);

            char json[128];
            snprintf(json, sizeof(json),
                     "{\"type\":\"oxygen_high_2\",\"value\":%.1f,\"threshold\":%.1f}",
                     data->oxygen_percent, s_rules.oxygen_level2);
            record_alarm(ALARM_LEVEL_WARN, "oxygen_high_2",
                         data->oxygen_percent, s_rules.oxygen_level2, json);
        }
    } else if (data->oxygen_percent < s_rules.oxygen_level2 - HYSTERESIS_O2) {
        s_alarm.o2_level2_active = false;
    }

    /* ── 一级预警 (O2 >= 22%): 关增氧，开进/排气阀换气 ── */
    if (data->oxygen_percent >= s_rules.oxygen_level1 &&
        !s_alarm.o2_level2_active && !s_alarm.o2_level3_active) {
        if (!s_alarm.o2_level1_active) {
            s_alarm.o2_level1_active = true;
            ESP_LOGW(TAG, "氧浓度一级预警: %.1f%% >= %.1f%%",
                     data->oxygen_percent, s_rules.oxygen_level1);

            /* 关增氧，低速换气（压力波动最小） */
            actuator_set(CH_OXYGEN_GEN,     false);
            actuator_set(CH_OXYGEN_PUMP,    false);
            actuator_set(CH_INTAKE_LOW,     true);      /* 低速进气 */
            actuator_set(CH_DEFLATE_NORM_O, true);      /* 正常泄气阀开 */
            actuator_set(CH_DEFLATE_NORM_C, false);

            voice_service_play(VOICE_ALARM_OXYGEN);

            char json[128];
            snprintf(json, sizeof(json),
                     "{\"type\":\"oxygen_high_1\",\"value\":%.1f,\"threshold\":%.1f}",
                     data->oxygen_percent, s_rules.oxygen_level1);
            record_alarm(ALARM_LEVEL_WARN, "oxygen_high_1",
                         data->oxygen_percent, s_rules.oxygen_level1, json);
        }
    } else if (data->oxygen_percent < s_rules.oxygen_level1 - HYSTERESIS_O2) {
        s_alarm.o2_level1_active = false;
    }
}

/**
 * @brief 压力检查（三级软件保护，从高到低判断）
 */
static void check_pressure(const sensor_data_t *data)
{
    /* ── 三级：极端超压 (>= 62KPa) → 紧急模式 ── */
    if (data->pressure_kpa >= s_rules.pressure_emerg) {
        if (!s_alarm.pressure_emerg_active) {
            s_alarm.pressure_emerg_active = true;
            ESP_LOGE(TAG, "压力极端超压: %.1fKPa >= %.1fKPa",
                     data->pressure_kpa, s_rules.pressure_emerg);

            char json[128];
            snprintf(json, sizeof(json),
                     "{\"type\":\"pressure_emerg\",\"value\":%.1f,\"threshold\":%.1f}",
                     data->pressure_kpa, s_rules.pressure_emerg);
            record_alarm(ALARM_LEVEL_FAULT, "pressure_emerg",
                         data->pressure_kpa, s_rules.pressure_emerg, NULL);
            emergency_shutdown("压力极端超压(>=62KPa)-紧急泄压", json);
        }
    } else if (data->pressure_kpa < s_rules.pressure_emerg - HYSTERESIS_PRESSURE) {
        s_alarm.pressure_emerg_active = false;
    }

    /* ── 二级：超限报警 (>= 60KPa) → 强制泄压 + ESTOP ── */
    if (data->pressure_kpa >= s_rules.pressure_fault && !s_alarm.pressure_emerg_active) {
        if (!s_alarm.pressure_fault_active) {
            s_alarm.pressure_fault_active = true;
            ESP_LOGE(TAG, "压力超限: %.1fKPa >= %.1fKPa",
                     data->pressure_kpa, s_rules.pressure_fault);

            /* 关闭所有进气阀 */
            actuator_set(CH_INTAKE_LOW,  false);
            actuator_set(CH_INTAKE_MED,  false);
            actuator_set(CH_INTAKE_FAST, false);

            /* 打开大流量排气阀强制泄压 */
            actuator_set(CH_DEFLATE_HIGH_O, true);
            actuator_set(CH_DEFLATE_HIGH_C, false);

            voice_service_play(VOICE_ALARM_PRESSURE);
            cabin_fsm_send_event(FSM_EVT_ESTOP);

            char json[128];
            snprintf(json, sizeof(json),
                     "{\"type\":\"pressure_fault\",\"value\":%.1f,\"threshold\":%.1f}",
                     data->pressure_kpa, s_rules.pressure_fault);
            record_alarm(ALARM_LEVEL_FAULT, "pressure_fault",
                         data->pressure_kpa, s_rules.pressure_fault, json);
        }
    } else if (data->pressure_kpa < s_rules.pressure_fault - HYSTERESIS_PRESSURE) {
        s_alarm.pressure_fault_active = false;
    }

    /* ── 一级：预警 (>= 58KPa) → 停止增压 ── */
    if (data->pressure_kpa >= s_rules.pressure_warn &&
        !s_alarm.pressure_fault_active && !s_alarm.pressure_emerg_active) {
        if (!s_alarm.pressure_warn_active) {
            s_alarm.pressure_warn_active = true;
            ESP_LOGW(TAG, "压力预警: %.1fKPa >= %.1fKPa",
                     data->pressure_kpa, s_rules.pressure_warn);

            /* 停止增压（关闭进气阀） */
            actuator_set(CH_INTAKE_LOW,  false);
            actuator_set(CH_INTAKE_MED,  false);
            actuator_set(CH_INTAKE_FAST, false);

            voice_service_play(VOICE_ALARM_PRESSURE);

            char json[128];
            snprintf(json, sizeof(json),
                     "{\"type\":\"pressure_warn\",\"value\":%.1f,\"threshold\":%.1f}",
                     data->pressure_kpa, s_rules.pressure_warn);
            record_alarm(ALARM_LEVEL_WARN, "pressure_warn",
                         data->pressure_kpa, s_rules.pressure_warn, json);
        }
    } else if (data->pressure_kpa < s_rules.pressure_warn - HYSTERESIS_PRESSURE) {
        s_alarm.pressure_warn_active = false;
    }
}

/**
 * @brief 压力变化率检查 (> 15 KPa/min → 停机保护)
 */
static void check_pressure_rate(const sensor_data_t *data)
{
    int64_t now_us = esp_timer_get_time();

    if (s_alarm.prev_pressure_time_us > 0) {
        float dt_min = (float)(now_us - s_alarm.prev_pressure_time_us) / 60000000.0f;

        if (dt_min > 0.01f) {   /* 避免除零 (最小 0.6ms) */
            float rate = fabsf(data->pressure_kpa - s_alarm.prev_pressure_kpa) / dt_min;

            if (rate > s_rules.pressure_rate_max) {
                if (!s_alarm.pressure_rate_active) {
                    s_alarm.pressure_rate_active = true;
                    ESP_LOGE(TAG, "压力变化率过快: %.1f KPa/min > %.1f KPa/min",
                             rate, s_rules.pressure_rate_max);

                    char json[160];
                    snprintf(json, sizeof(json),
                             "{\"type\":\"pressure_rate\",\"value\":%.1f,\"threshold\":%.1f}",
                             rate, s_rules.pressure_rate_max);
                    record_alarm(ALARM_LEVEL_FAULT, "pressure_rate",
                                 rate, s_rules.pressure_rate_max, NULL);
                    emergency_shutdown("压力变化率过快(>15KPa/min)", json);
                }
            } else {
                s_alarm.pressure_rate_active = false;
            }
        }
    }

    /* 更新上一次的压力和时间 */
    s_alarm.prev_pressure_kpa     = data->pressure_kpa;
    s_alarm.prev_pressure_time_us = now_us;
}

/* ════════════════════════════════════════════════
 * 规则引擎主任务
 * ════════════════════════════════════════════════ */

static void rule_task(void *arg)
{
    ESP_LOGI(TAG, "规则引擎任务启动");

    while (1) {
        /* 等待传感器数据就绪事件 */
        EventBits_t bits = xEventGroupWaitBits(
            g_system_events,
            EVT_SENSOR_DATA_READY,
            pdTRUE,         /* 自动清除 */
            pdFALSE,
            portMAX_DELAY
        );

        if (!(bits & EVT_SENSOR_DATA_READY)) {
            continue;
        }

        /* 读取最新传感器数据 */
        sensor_data_t data;
        esp_err_t ret = sensor_service_get_data(&data);

        /* ════ 1. 传感器离线检测 ════ */
        if (ret != ESP_OK || !data.valid) {
            s_alarm.sensor_fail_count++;
            ESP_LOGW(TAG, "传感器数据无效 (连续 %d 次)", s_alarm.sensor_fail_count);

            if (s_alarm.sensor_fail_count >= s_rules.sensor_timeout) {
                char json[128];
                snprintf(json, sizeof(json),
                         "{\"type\":\"sensor_offline\",\"count\":%d,\"threshold\":%d}",
                         s_alarm.sensor_fail_count, s_rules.sensor_timeout);
                record_alarm(ALARM_LEVEL_FAULT, "sensor_offline",
                             (float)s_alarm.sensor_fail_count,
                             (float)s_rules.sensor_timeout, NULL);
                emergency_shutdown("传感器故障-连续超时", json);
            }
            continue;   /* 数据无效，跳过后续阈值检查 */
        }
        s_alarm.sensor_fail_count = 0;

        /* ════ 2. 温度检查（两级） ════ */
        check_temperature(&data);

        /* ════ 3. 湿度检查（高低两端） ════ */
        check_humidity(&data);

        /* ════ 4. 氧浓度检查（三级高氧方向） ════ */
        check_oxygen(&data);

        /* ════ 5. 压力检查（三级软件） ════ */
        check_pressure(&data);

        /* ════ 6. 压力变化率检查 ════ */
        check_pressure_rate(&data);
    }
}

/* ════════════════════════════════════════════════
 * 公共 API
 * ════════════════════════════════════════════════ */

esp_err_t rule_engine_init(void)
{
    /* 先设置代码内置默认值 */
    s_rules.temp_warn         = APP_CFG_ALARM_TEMP_WARN;
    s_rules.temp_fault        = APP_CFG_ALARM_TEMP_FAULT;
    s_rules.humidity_low      = APP_CFG_ALARM_HUMIDITY_LOW;
    s_rules.humidity_high     = APP_CFG_ALARM_HUMIDITY_HIGH;
    s_rules.oxygen_level1     = APP_CFG_ALARM_O2_LEVEL1;
    s_rules.oxygen_level2     = APP_CFG_ALARM_O2_LEVEL2;
    s_rules.oxygen_level3     = APP_CFG_ALARM_O2_LEVEL3;
    s_rules.pressure_warn     = APP_CFG_ALARM_PRESSURE_WARN;
    s_rules.pressure_fault    = APP_CFG_ALARM_PRESSURE_FAULT;
    s_rules.pressure_emerg    = APP_CFG_ALARM_PRESSURE_EMERG;
    s_rules.pressure_rate_max = APP_CFG_ALARM_PRESSURE_RATE;
    s_rules.sensor_timeout    = APP_CFG_ALARM_SENSOR_TIMEOUT;

    /* 尝试从 NVS 加载已保存的规则（覆盖默认值） */
    alarm_rules_t nvs_rules;
    size_t len = sizeof(nvs_rules);
    if (data_store_get_blob("alarm_rules", &nvs_rules, &len) == ESP_OK &&
        len == sizeof(alarm_rules_t)) {
        s_rules = nvs_rules;
        ESP_LOGI(TAG, "从 NVS 加载报警规则成功");
    } else {
        ESP_LOGI(TAG, "使用默认报警阈值 (NVS 无数据或版本不匹配)");
    }

    /* 清空报警状态 */
    memset(&s_alarm, 0, sizeof(s_alarm));

    /* 创建报警历史缓冲的互斥锁 (供 record_alarm 写 + get_recent 读 同步) */
    if (s_alarm_log.mutex == NULL) {
        s_alarm_log.mutex = xSemaphoreCreateMutex();
        if (s_alarm_log.mutex == NULL) {
            ESP_LOGE(TAG, "创建报警历史互斥锁失败");
            /* 非致命: 历史记录会失败但报警分发本身不受影响 */
        }
    }
    s_alarm_log.write_idx = 0;
    s_alarm_log.count     = 0;

    ESP_LOGI(TAG, "规则引擎初始化完成:");
    ESP_LOGI(TAG, "  温度: 警告>%.0f℃, 故障>%.0f℃", s_rules.temp_warn, s_rules.temp_fault);
    ESP_LOGI(TAG, "  湿度: 低<%.0f%%RH, 高>%.0f%%RH", s_rules.humidity_low, s_rules.humidity_high);
    ESP_LOGI(TAG, "  氧浓度: L1>=%.0f%%, L2>=%.0f%%, L3>=%.0f%%",
             s_rules.oxygen_level1, s_rules.oxygen_level2, s_rules.oxygen_level3);
    ESP_LOGI(TAG, "  压力: 预警>=%.0fKPa, 超限>=%.0fKPa, 极端>=%.0fKPa",
             s_rules.pressure_warn, s_rules.pressure_fault, s_rules.pressure_emerg);
    ESP_LOGI(TAG, "  变化率: >%.0f KPa/min, 传感器超时: %d 次",
             s_rules.pressure_rate_max, s_rules.sensor_timeout);

    return ESP_OK;
}

esp_err_t rule_engine_start(void)
{
    BaseType_t ret = xTaskCreate(rule_task, "rule_task", 4096, NULL, 4, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 rule_task 失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "规则引擎任务已启动 (优先级=4)");
    return ESP_OK;
}

esp_err_t rule_engine_get_recent(alarm_record_t *out, uint8_t max, uint8_t *count)
{
    if (out == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *count = 0;
    if (max == 0 || s_alarm_log.mutex == NULL) {
        return ESP_OK;
    }

    if (xSemaphoreTake(s_alarm_log.mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "get_recent 获取锁超时");
        return ESP_ERR_TIMEOUT;
    }

    /* 环形缓冲读取顺序: 最新 → 最旧
     * write_idx 指向"下一个写入位置", 所以最新一条在 (write_idx-1+CAP)%CAP
     * count 是当前有效条数 (≤ CAPACITY)
     * 输出条数 = min(max, count)
     */
    uint8_t n = (max < s_alarm_log.count) ? max : s_alarm_log.count;
    for (uint8_t i = 0; i < n; i++) {
        /* 第 i 条 (从最新算起) 在环形缓冲中的下标 */
        uint8_t idx = (s_alarm_log.write_idx + ALARM_HISTORY_CAPACITY - 1 - i)
                      % ALARM_HISTORY_CAPACITY;
        out[i] = s_alarm_log.ring[idx];
    }
    *count = n;

    xSemaphoreGive(s_alarm_log.mutex);
    return ESP_OK;
}
