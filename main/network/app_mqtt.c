/**
 * @file app_mqtt.c
 * @brief MQTT 客户端通信实现
 *
 * 连接目标：EMQX Cloud (ie1f9c8f.ala.cn-hangzhou.emqxsl.cn:8883, TLS)
 * 数据流：ESP32 → EMQX Cloud → 规则引擎 HTTP → cpolar → ThingsBoard
 *
 * TLS 验证使用 ESP-IDF 内置 Mozilla 根证书包（sdkconfig 已开启
 * CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y / DEFAULT_FULL=y）。
 *
 * 连接成功后：
 *   1. 订阅命令主题 APP_CFG_MQTT_CMD_TOPIC 和参数主题 APP_CFG_MQTT_PARAM_TOPIC
 *   2. 发布 retain status（在线通知）
 *   3. 启动 telemetry_task，等待 EVT_SENSOR_DATA_READY，发布 4 字段 JSON
 *
 * EMQX 规则引擎 SQL: SELECT * FROM "oxy/device/+/data"
 * TB 推送 URL: http://oxysys.cpolar.top/api/v1/GFdaO9KdILx0P8KOQQ0T/telemetry
 * TB Body: {"pressure":${payload.pressure},"oxygen":${payload.oxygen},
 *           "temperature":${payload.temperature},"humidity":${payload.humidity}}
 */
#include "network/app_mqtt.h"
#include <mqtt_client.h>          /* ESP-IDF MQTT 客户端（系统路径，尖括号） */
#include "esp_crt_bundle.h"
#include "common/app_config.h"
#include "common/app_events.h"
#include "services/sensor_service.h"
#include "services/cabin_fsm.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

static const char *TAG = "mqtt";

static esp_mqtt_client_handle_t s_client = NULL;
static mqtt_cmd_cb_t s_cmd_cb = NULL;
static bool s_mqtt_connected = false;

/* ==================== 内部辅助 ==================== */

/**
 * @brief 发布 status 消息（retain=true，连接时调用一次，状态变化时也可调用）
 */
esp_err_t mqtt_publish_status(const char *state_name)
{
    if (!s_mqtt_connected || s_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    char payload[384];
    /* SNTP 同步后使用真实 Unix 毫秒时间戳，未同步时 time() 返回 0 附近的值 */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t ts_ms = (int64_t)tv.tv_sec * 1000LL + tv.tv_usec / 1000LL;

    /* 获取运行详情 */
    cabin_run_info_t run_info = {0};
    cabin_fsm_get_run_info(&run_info);
    const char *phase_name = cabin_fsm_get_phase_name();

    snprintf(payload, sizeof(payload),
             "{\"online\":true,\"firmware\":\"%s\",\"state\":\"%s\","
             "\"state_detail\":{\"phase\":\"%s\","
             "\"target_kpa\":%.1f,\"hold_min\":%lu,"
             "\"elapsed_min\":%lu,\"remaining_min\":%lu},"
             "\"ts\":%lld}",
             APP_CFG_FIRMWARE_VERSION,
             state_name ? state_name : "unknown",
             phase_name,
             (double)run_info.target_pressure_kpa,
             (unsigned long)run_info.hold_time_min,
             (unsigned long)run_info.elapsed_min,
             (unsigned long)run_info.remaining_min,
             (long long)ts_ms);

    int msg_id = esp_mqtt_client_publish(
        s_client,
        APP_CFG_MQTT_STATUS_TOPIC,
        payload,
        0,   /* len=0: 使用 strlen */
        1,   /* QoS 1 */
        1    /* retain=1: 新订阅者可收到最后一条状态 */
    );

    if (msg_id >= 0) {
        ESP_LOGI(TAG, "状态上报: %s", payload);
    }
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

/* ==================== Telemetry 上报任务 ==================== */

/**
 * @brief MQTT Telemetry 上报任务
 *
 * 传感器采集频率（APP_CFG_MODBUS_POLL_MS = 2s）与 MQTT 上报频率
 * （APP_CFG_MQTT_REPORT_INTERVAL_MS = 10s）**解耦**：
 *
 *   - sensor_service 每 2s 采集一次并设置 EVT_SENSOR_DATA_READY
 *   - rule_engine / screen_service 依赖 2s 采集频率（阈值检测、屏幕刷新）
 *   - telemetry_task 每次收到 EVT_SENSOR_DATA_READY 都更新本地缓存，
 *     但只在距上次上报满 10s 后才真正 publish，避免频繁占用 MQTT 带宽
 *
 * 如需调整上报频率，仅修改 app_config.h 中的
 * APP_CFG_MQTT_REPORT_INTERVAL_MS 即可，无需改动此函数。
 */
static void telemetry_task(void *arg)
{
    char payload[192];
    sensor_data_t data;
    int64_t last_report_us = 0;     /* 上次成功上报的时间（esp_timer 微秒） */
    const int64_t interval_us = (int64_t)APP_CFG_MQTT_REPORT_INTERVAL_MS * 1000LL;

    ESP_LOGI(TAG, "Telemetry 上报任务启动 (上报间隔 %d ms，采集间隔 %d ms)",
             APP_CFG_MQTT_REPORT_INTERVAL_MS, APP_CFG_MODBUS_POLL_MS);

    while (1) {
        if (g_system_events == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* 等待传感器数据就绪（超时 APP_CFG_MODBUS_POLL_MS + 1s，保持同步） */
        EventBits_t bits = xEventGroupWaitBits(
            g_system_events,
            EVT_SENSOR_DATA_READY,
            pdTRUE,    /* 清除事件位 */
            pdFALSE,
            pdMS_TO_TICKS(APP_CFG_MODBUS_POLL_MS + 1000)
        );

        if (!(bits & EVT_SENSOR_DATA_READY)) {
            continue;  /* 超时：传感器离线或初始化中，继续等待 */
        }

        /* 限速：距上次上报未满 interval_us，跳过本次 publish */
        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_report_us) < interval_us) {
            continue;
        }

        if (!s_mqtt_connected || s_client == NULL) {
            ESP_LOGD(TAG, "MQTT 未连接，跳过本次上报");
            continue;
        }

        if (sensor_service_get_data(&data) != ESP_OK || !data.valid) {
            ESP_LOGD(TAG, "传感器数据无效，跳过本次上报");
            continue;
        }

        /* 4 字段 JSON，与 EMQX 规则引擎 Body 模板变量名完全对应 */
        snprintf(payload, sizeof(payload),
                 "{\"pressure\":%.1f,\"oxygen\":%.1f,"
                 "\"temperature\":%.1f,\"humidity\":%.1f}",
                 data.pressure_kpa,
                 data.oxygen_percent,
                 data.temperature,
                 data.humidity);

        int msg_id = esp_mqtt_client_publish(
            s_client,
            APP_CFG_MQTT_REPORT_TOPIC,
            payload,
            0,   /* len=0: 使用 strlen */
            1,   /* QoS 1 */
            0    /* retain=0 */
        );

        if (msg_id >= 0) {
            last_report_us = now_us;  /* 更新上报时间戳 */
            ESP_LOGI(TAG, "上报 [%d]: %s", msg_id, payload);
        } else {
            ESP_LOGW(TAG, "上报失败，下次重试");
            /* 上报失败：不更新 last_report_us，下一个采集周期立即重试 */
        }
    }
}

/* ==================== CMD 解析 ==================== */

/**
 * @brief 解析并分发 cmd 指令到状态机
 *
 * 支持的 cmd 值（与主控端规范对齐）：
 *   start / stop / pause / resume / emergency_stop / ventilate / disinfect
 */
static void dispatch_cmd(const char *payload_str)
{
    /* 简单字符串匹配（无 cJSON 依赖）
     * payload 示例: {"cmd":"start","params":{},"ts":1711612345678} */
    if      (strstr(payload_str, "\"start\""))          cabin_fsm_send_event(FSM_EVT_START);
    else if (strstr(payload_str, "\"stop\""))           cabin_fsm_send_event(FSM_EVT_RESET);
    else if (strstr(payload_str, "\"pause\""))          cabin_fsm_send_event(FSM_EVT_PAUSE);
    else if (strstr(payload_str, "\"resume\""))         cabin_fsm_send_event(FSM_EVT_RESUME);
    else if (strstr(payload_str, "\"emergency_stop\"")) cabin_fsm_send_event(FSM_EVT_ESTOP);
    else if (strstr(payload_str, "\"ventilate\""))      cabin_fsm_send_event(FSM_EVT_VENTILATE);
    else if (strstr(payload_str, "\"disinfect\""))      cabin_fsm_send_event(FSM_EVT_DISINFECT);
    else {
        ESP_LOGW(TAG, "未知 cmd: %s", payload_str);
    }
}

/* ==================== MQTT 事件处理 ==================== */

static void on_mqtt_event(void *handler_args, esp_event_base_t base,
                          int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "已连接到 EMQX Cloud");
            s_mqtt_connected = true;
            if (g_system_events) {
                xEventGroupSetBits(g_system_events,   EVT_MQTT_CONNECTED);
                xEventGroupClearBits(g_system_events, EVT_MQTT_DISCONNECTED);
            }
            /* 订阅控制指令主题 */
            esp_mqtt_client_subscribe(s_client, APP_CFG_MQTT_CMD_TOPIC,   1);
            esp_mqtt_client_subscribe(s_client, APP_CFG_MQTT_PARAM_TOPIC, 1);
            ESP_LOGI(TAG, "已订阅: %s / %s",
                     APP_CFG_MQTT_CMD_TOPIC, APP_CFG_MQTT_PARAM_TOPIC);
            /* 发布上线状态（retain，小程序/主控可立即感知设备在线） */
            mqtt_publish_status(cabin_fsm_get_state_name());
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT 断线，等待自动重连...");
            s_mqtt_connected = false;
            if (g_system_events) {
                xEventGroupClearBits(g_system_events, EVT_MQTT_CONNECTED);
                xEventGroupSetBits(g_system_events,   EVT_MQTT_DISCONNECTED);
            }
            break;

        case MQTT_EVENT_DATA: {
            char topic[80]       = {0};
            char payload_buf[512] = {0};
            int tlen = (event->topic_len  < 79)  ? event->topic_len  : 79;
            int plen = (event->data_len   < 511) ? event->data_len   : 511;
            memcpy(topic,       event->topic, tlen);
            memcpy(payload_buf, event->data,  plen);
            ESP_LOGI(TAG, "收到 [%s]: %s", topic, payload_buf);

            /* cmd 主题：解析并发送到状态机 */
            if (strstr(topic, "/cmd")) {
                dispatch_cmd(payload_buf);
            }
            /* param 主题：交给外部回调处理 */
            if (s_cmd_cb) {
                s_cmd_cb(topic, payload_buf);
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT 错误事件");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "  esp_tls 错误: 0x%x",
                         event->error_handle->esp_tls_last_esp_err);
                ESP_LOGE(TAG, "  TLS 堆栈错误: 0x%x",
                         event->error_handle->esp_tls_stack_err);
            }
            break;

        default:
            break;
    }
}

/* ==================== 公共 API ==================== */

esp_err_t mqtt_client_init(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker = {
            .address = {
                .uri = APP_CFG_MQTT_BROKER_URI,
            },
            .verification = {
                .crt_bundle_attach = esp_crt_bundle_attach,
            },
        },
        .credentials = {
            .username  = APP_CFG_MQTT_USERNAME,
            .client_id = APP_CFG_MQTT_CLIENT_ID,
            .authentication = {
                .password = APP_CFG_MQTT_PASSWORD,
            },
        },
        .network = {
            .reconnect_timeout_ms = 5000,
        },
        .session = {
            .keepalive = 60,
        },
    };

    s_client = esp_mqtt_client_init(&cfg);
    if (s_client == NULL) {
        ESP_LOGE(TAG, "esp_mqtt_client_init 失败");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        s_client, ESP_EVENT_ANY_ID, on_mqtt_event, NULL));

    ESP_ERROR_CHECK(esp_mqtt_client_start(s_client));

    /* 创建 telemetry 上报任务（优先级 4，栈 4096） */
    BaseType_t ret = xTaskCreate(telemetry_task, "telemetry_task",
                                 4096, NULL, 4, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 telemetry_task 失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MQTT 初始化完成 → %s", APP_CFG_MQTT_BROKER_URI);
    return ESP_OK;
}

esp_err_t mqtt_publish_sensor_data(const char *topic, const char *payload)
{
    if (!s_mqtt_connected || s_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    int msg_id = esp_mqtt_client_publish(s_client, topic, payload, 0, 1, 0);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t mqtt_publish_alarm(const char *topic, const char *payload)
{
    if (!s_mqtt_connected || s_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /* QoS=1, retain=1 */
    int msg_id = esp_mqtt_client_publish(s_client, topic, payload, 0, 1, 1);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t mqtt_register_cmd_callback(mqtt_cmd_cb_t cb)
{
    s_cmd_cb = cb;
    return ESP_OK;
}
