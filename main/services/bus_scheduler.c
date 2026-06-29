/**
 * @file bus_scheduler.c
 * @brief RS485 总线分时调度器实现
 *
 * 互斥锁 g_rs485_mutex 已在 app_events_init() 中创建，
 * 本模块仅验证其有效性并封装获取/释放操作。
 */
#include "bus_scheduler.h"
#include "common/app_events.h"
#include "esp_log.h"

static const char *TAG = "bus_sched";

/* 当前占用者（仅用于调试日志） */
static bus_user_t s_current_user;

static const char *user_name(bus_user_t user)
{
    return user == BUS_USER_SENSOR ? "传感器" : "串口屏";
}

esp_err_t bus_scheduler_init(void)
{
    /* g_rs485_mutex 已由 app_events_init() 创建，这里只做校验 */
    if (g_rs485_mutex == NULL) {
        ESP_LOGE(TAG, "g_rs485_mutex 为空，请确认 app_events_init() 已先调用");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RS485 总线调度器初始化完成");
    return ESP_OK;
}

esp_err_t bus_acquire(bus_user_t user, uint32_t timeout_ms)
{
    if (xSemaphoreTake(g_rs485_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        s_current_user = user;
        ESP_LOGD(TAG, "485 总线被 %s 占用", user_name(user));
        return ESP_OK;
    }
    /* 非阻塞/短超时场景下, 总线被另一方持有是"正常竞争", 用 DEBUG 级避免刷屏。
     * 长超时场景(如 > 1s)再超时才是异常, 上层调用方自己决定要不要报 W 级。*/
    ESP_LOGD(TAG, "%s 获取 485 总线超时 (当前占用: %s)",
             user_name(user), user_name(s_current_user));
    return ESP_ERR_TIMEOUT;
}

esp_err_t bus_release(bus_user_t user)
{
    ESP_LOGD(TAG, "485 总线被 %s 释放", user_name(user));
    xSemaphoreGive(g_rs485_mutex);
    return ESP_OK;
}
