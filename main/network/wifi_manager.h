/**
 * @file wifi_manager.h
 * @brief WiFi 连接管理
 *
 * 【职责】
 *   - 管理 ESP32-S3 的 WiFi Station 连接
 *   - 从 NVS 读取已保存的 SSID/密码，自动连接
 *   - 断线自动重连
 *   - 提供连接状态查询接口
 *
 * 【WiFi 凭据来源】
 *   1. NVS 已保存的凭据（上次成功连接的 SSID/密码）
 *   2. 串口屏输入（用户通过触控屏输入 SSID/密码）
 *   3. UART0 串口命令（调试用）
 *
 * 【事件通知】
 *   连接成功 → 设置 EVT_WIFI_CONNECTED，清除 EVT_WIFI_DISCONNECTED
 *   断线     → 设置 EVT_WIFI_DISCONNECTED，清除 EVT_WIFI_CONNECTED
 *   其他服务（MQTT、SNTP）通过监听事件组来感知 WiFi 状态变化
 */
#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief 初始化 WiFi Station 模式
 *
 * 步骤：
 *   1. 初始化 TCP/IP 适配器和默认事件循环
 *   2. 创建默认 WiFi Station
 *   3. 注册 WiFi 事件处理（WIFI_EVENT / IP_EVENT）
 *   4. 从 NVS 读取 SSID/密码
 *   5. 启动 WiFi 连接
 *
 * @return ESP_OK 成功
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief 设置新的 WiFi 凭据并重新连接
 *
 * 保存到 NVS 后断开当前连接，使用新凭据重连。
 * 来源：串口屏 WiFi 配置页面、MQTT 远程配置。
 *
 * @param ssid     WiFi SSID（最长 32 字符）
 * @param password WiFi 密码（最长 64 字符）
 * @return ESP_OK 成功
 */
esp_err_t wifi_manager_set_credentials(const char *ssid, const char *password);

/**
 * @brief 查询当前 WiFi 连接状态
 * @return true=已连接且获得 IP, false=未连接
 */
bool wifi_manager_is_connected(void);

#endif // WIFI_MANAGER_H
