/**
 * @file wifi_manager.c
 * @brief WiFi 连接管理实现（支持主/备 WiFi 自动切换）
 *
 * 当前阶段：SSID/密码硬编码在 app_config.h 中（串口屏未接入）。
 * 后续可通过 wifi_manager_set_credentials() 动态更新并保存到 NVS。
 *
 * 【主/备 WiFi 切换策略】
 *   - 主 WiFi: APP_CFG_WIFI_SSID / APP_CFG_WIFI_PASSWORD
 *   - 备 WiFi: APP_CFG_WIFI_SSID_BACKUP / APP_CFG_WIFI_PASSWORD_BACKUP
 *   - 连续断线 APP_CFG_WIFI_SWITCH_THRESHOLD 次后自动切换到另一组
 *   - 连接成功后重置失败计数
 */
#include "wifi_manager.h"
#include "common/app_config.h"
#include "common/app_events.h"
#include "common/data_store.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char *TAG = "wifi_mgr";
static bool s_connected = false;

/* ==================== 主/备 WiFi 切换管理 ==================== */

/* WiFi 凭据表 */
typedef struct {
    const char *ssid;
    const char *password;
} wifi_credential_t;

static const wifi_credential_t s_wifi_list[] = {
    { APP_CFG_WIFI_SSID,        APP_CFG_WIFI_PASSWORD        },  /* 索引 0: 主 WiFi */
    { APP_CFG_WIFI_SSID_BACKUP, APP_CFG_WIFI_PASSWORD_BACKUP },  /* 索引 1: 备用 WiFi */
};
#define WIFI_LIST_COUNT  (sizeof(s_wifi_list) / sizeof(s_wifi_list[0]))

static uint8_t  s_current_idx  = 0;   /* 当前使用的 WiFi 索引 */
static uint8_t  s_fail_count   = 0;   /* 连续断线计数 */

/**
 * @brief 切换到下一组 WiFi 凭据并发起连接
 */
static void wifi_switch_and_connect(void)
{
    s_current_idx = (s_current_idx + 1) % WIFI_LIST_COUNT;
    s_fail_count = 0;

    const wifi_credential_t *cred = &s_wifi_list[s_current_idx];

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid,     cred->ssid,     sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password,  cred->password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable  = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_LOGW(TAG, "切换到 WiFi[%d]: %s", s_current_idx, cred->ssid);

    esp_wifi_disconnect();
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_wifi_connect();
}

/* ==================== WiFi 事件处理 ==================== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi STA 已启动，连接 WiFi[%d]: %s",
                         s_current_idx, s_wifi_list[s_current_idx].ssid);
                esp_wifi_connect();
                break;

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "WiFi 已连接，等待获取 IP...");
                break;

            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t *evt =
                    (wifi_event_sta_disconnected_t *)event_data;
                s_connected = false;
                if (g_system_events) {
                    xEventGroupClearBits(g_system_events, EVT_WIFI_CONNECTED);
                    xEventGroupSetBits(g_system_events, EVT_WIFI_DISCONNECTED);
                }

                s_fail_count++;
                ESP_LOGW(TAG, "WiFi[%d] 断线 (reason=%d)，失败次数: %d/%d",
                         s_current_idx, evt->reason,
                         s_fail_count, APP_CFG_WIFI_SWITCH_THRESHOLD);

                if (s_fail_count >= APP_CFG_WIFI_SWITCH_THRESHOLD) {
                    /* 连续失败达到阈值，切换到另一组 WiFi */
                    wifi_switch_and_connect();
                } else {
                    /* 未达阈值，继续重连当前 WiFi */
                    esp_wifi_connect();
                }
                break;
            }

            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_connected = true;
        s_fail_count = 0;  /* 连接成功，重置失败计数 */
        if (g_system_events) {
            xEventGroupClearBits(g_system_events, EVT_WIFI_DISCONNECTED);
            xEventGroupSetBits(g_system_events, EVT_WIFI_CONNECTED);
        }
        ESP_LOGI(TAG, "WiFi[%d] 已获取 IP: " IPSTR, s_current_idx, IP2STR(&event->ip_info.ip));
    }
}

/* ==================== 公共 API ==================== */

esp_err_t wifi_manager_init(void)
{
    /* 1. 初始化网络接口和事件循环（全局唯一，幂等） */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /* 2. 初始化 WiFi 驱动 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* 3. 注册事件处理器 */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    /* 4. 配置 WiFi Station — 从主WiFi开始 */
    const wifi_credential_t *cred = &s_wifi_list[s_current_idx];

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid,     cred->ssid,     sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password,  cred->password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable  = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi 初始化完成，主WiFi: %s / 备用WiFi: %s",
             s_wifi_list[0].ssid, s_wifi_list[1].ssid);
    return ESP_OK;
}

esp_err_t wifi_manager_set_credentials(const char *ssid, const char *password)
{
    data_store_set_str("wifi_ssid", ssid);
    data_store_set_str("wifi_pass", password);

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid,     ssid,     sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    esp_wifi_disconnect();
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_wifi_connect();

    ESP_LOGI(TAG, "WiFi 凭据已更新: %s", ssid);
    return ESP_OK;
}

bool wifi_manager_is_connected(void)
{
    return s_connected;
}
