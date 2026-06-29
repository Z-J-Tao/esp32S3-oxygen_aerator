/**
 * @file app_mqtt.h
 * @brief MQTT 客户端通信（项目封装层）
 *
 * 【职责】
 *   - 连接 MQTT 服务器（EMQX Cloud TLS/8883）
 *   - 定期发布传感器数据（数据上报）
 *   - 发布设备状态（status retain）
 *   - 发布告警信息
 *   - 订阅远程控制主题，接收并分发指令
 *
 * 【MQTT 主题设计】（前缀 oxy/device/{deviceId}/）
 *   发布:
 *     oxy/device/ESP32S3_001/data    — 传感器实时数据（JSON，10s/次）
 *     oxy/device/ESP32S3_001/status  — 设备状态（JSON，retain=true）
 *     oxy/device/ESP32S3_001/alert   — 告警信息（JSON，QoS=1 retain=true）
 *   订阅:
 *     oxy/device/ESP32S3_001/cmd     — 远程控制指令
 *     oxy/device/ESP32S3_001/param   — 参数设置下发
 *
 * 【数据格式】
 *   传感器上报 (data)：
 *     {"pressure": 15.2, "oxygen": 21.0, "temperature": 24.1, "humidity": 50.3}
 *   设备状态 (status)：
 *     {"online": true, "firmware": "1.0.0", "state": "holding_mid",
 *      "state_detail": {...}, "ts": 1711612345678}
 *   告警 (alert)：
 *     {"type": "temp_high", "value": 42.1, "threshold": 40.0}
 *   控制指令 (cmd，支持的 cmd 值)：
 *     start / stop / pause / resume / emergency_stop / ventilate / disinfect
 *
 * 【命名说明】
 *   本文件命名为 app_mqtt.h（而非 mqtt_client.h），以避免与
 *   ESP-IDF 组件 mqtt 中的同名头文件 <mqtt_client.h> 冲突。
 */
#ifndef APP_MQTT_H
#define APP_MQTT_H

#include "esp_err.h"

/**
 * @brief 初始化 MQTT 客户端
 *
 * 配置 broker URI（EMQX Cloud TLS），注册事件处理，启动客户端。
 * 实际连接在 WiFi 连接成功后自动发起。
 * 同时启动 telemetry_task，等待传感器数据并上报到 EMQX Cloud。
 *
 * @return ESP_OK 成功
 */
esp_err_t mqtt_client_init(void);

/**
 * @brief 发布传感器数据（由 telemetry_task 自动调用）
 *
 * @param topic   发布主题（如 APP_CFG_MQTT_REPORT_TOPIC）
 * @param payload JSON 格式的传感器数据字符串
 * @return ESP_OK 成功，ESP_ERR_INVALID_STATE 未连接
 */
esp_err_t mqtt_publish_sensor_data(const char *topic, const char *payload);

/**
 * @brief 发布设备状态（retain=true，QoS=1）
 *
 * 在 MQTT 连接建立时自动调用一次（上线通知），
 * 也可由状态机在状态变化时主动调用。
 *
 * @param state_name  状态字符串（由 cabin_fsm_get_state_name() 获取）
 * @return ESP_OK 成功
 */
esp_err_t mqtt_publish_status(const char *state_name);

/**
 * @brief 发布告警信息（QoS=1, retain=1）
 *
 * @param topic   发布主题（如 APP_CFG_MQTT_ALERT_TOPIC）
 * @param payload 告警内容字符串
 * @return ESP_OK 成功
 */
esp_err_t mqtt_publish_alarm(const char *topic, const char *payload);

/**
 * @brief 远程控制指令回调类型
 *
 * 支持的 cmd 值：start / stop / pause / resume / emergency_stop / ventilate / disinfect
 */
typedef void (*mqtt_cmd_cb_t)(const char *topic, const char *payload);

/**
 * @brief 注册远程控制指令回调
 *
 * @param cb 回调函数指针
 * @return ESP_OK 成功
 */
esp_err_t mqtt_register_cmd_callback(mqtt_cmd_cb_t cb);

#endif // APP_MQTT_H
