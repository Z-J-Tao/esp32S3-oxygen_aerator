/**
 * @file data_store.h
 * @brief NVS 持久化存储封装
 *
 * 【职责】
 *   封装 ESP-IDF 的 NVS (Non-Volatile Storage) API，
 *   提供简单的 key-value 存取接口，用于掉电保存：
 *     - WiFi SSID/密码         (字符串)
 *     - MQTT broker 地址       (字符串)
 *     - 报警阈值规则           (blob)
 *     - 定时任务列表           (blob)
 *     - 其他用户配置           (int32/字符串)
 *
 * 【NVS 命名空间】
 *   使用统一的命名空间 "app_store"（建议在实现中定义）。
 *
 * 【常用 key 列表】
 *   "wifi_ssid"    — WiFi SSID (字符串, 最长 32)
 *   "wifi_pass"    — WiFi 密码 (字符串, 最长 64)
 *   "mqtt_uri"     — MQTT 服务器地址 (字符串)
 *   "alarm_rules"  — 报警规则 (blob)
 *   "timer_tasks"  — 定时任务列表 (blob)
 *
 * 【注意事项】
 *   - NVS 有写入次数限制（Flash 寿命），避免高频写入
 *   - blob 数据大小建议 < 4KB
 *   - 首次使用（NVS 为空）时，各模块应使用默认值
 */
#ifndef DATA_STORE_H
#define DATA_STORE_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

/**
 * @brief 初始化 NVS Flash
 *
 * 调用 nvs_flash_init()，如果 NVS 分区被截断或格式不对，
 * 会执行 nvs_flash_erase() 后重新初始化。
 *
 * @return ESP_OK 成功
 */
esp_err_t data_store_init(void);

/**
 * @brief 读取字符串
 * @param key     键名
 * @param value   [out] 目标缓冲区
 * @param max_len 缓冲区最大长度
 * @return ESP_OK 成功，ESP_ERR_NVS_NOT_FOUND 键不存在
 */
esp_err_t data_store_get_str(const char *key, char *value, size_t max_len);

/**
 * @brief 写入字符串
 * @param key   键名
 * @param value 要写入的字符串（以 '\0' 结尾）
 * @return ESP_OK 成功
 */
esp_err_t data_store_set_str(const char *key, const char *value);

/**
 * @brief 读取 32 位整数
 * @param key   键名
 * @param value [out] 读取的值
 * @return ESP_OK 成功，ESP_ERR_NVS_NOT_FOUND 键不存在
 */
esp_err_t data_store_get_i32(const char *key, int32_t *value);

/**
 * @brief 写入 32 位整数
 * @param key   键名
 * @param value 要写入的值
 * @return ESP_OK 成功
 */
esp_err_t data_store_set_i32(const char *key, int32_t value);

/**
 * @brief 读取二进制数据块
 *
 * 用于存取结构体数组（如报警规则表、定时任务表）。
 *
 * @param key    键名
 * @param data   [out] 目标缓冲区
 * @param length [in/out] 输入=缓冲区大小，输出=实际读取长度
 * @return ESP_OK 成功，ESP_ERR_NVS_NOT_FOUND 键不存在
 */
esp_err_t data_store_get_blob(const char *key, void *data, size_t *length);

/**
 * @brief 写入二进制数据块
 * @param key    键名
 * @param data   要写入的数据
 * @param length 数据长度（字节）
 * @return ESP_OK 成功
 */
esp_err_t data_store_set_blob(const char *key, const void *data, size_t length);

#endif // DATA_STORE_H
