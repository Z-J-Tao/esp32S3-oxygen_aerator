/**
 * @file bsp_gpio.c
 * @brief 通用 GPIO 初始化实现
 *
 * 引脚定义参见 common/app_config.h，与原理图 Schematic3 V3.0 对应。
 * GPIO48: 调试 LED（推挽输出）
 * GPIO3:  应急按钮（输入上拉，下降沿中断，50ms 软件消抖）
 */
#include "bsp_gpio.h"
#include "common/app_config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "bsp_gpio";

/* 应急按钮回调 */
static emergency_btn_cb_t s_emergency_cb = NULL;

/* ISR 消抖：记录上次触发的时间戳（微秒） */
static volatile int64_t s_last_isr_time_us = 0;

/**
 * @brief 应急按钮中断服务函数（下降沿触发）
 *
 * 消抖策略：比较当前时间与上次触发时间，间隔 < 50ms 则忽略。
 * esp_timer_get_time() 在 ESP-IDF v5.x 中是 IRAM 安全的。
 */
static void IRAM_ATTR emergency_isr_handler(void *arg)
{
    int64_t now_us = esp_timer_get_time();
    if ((now_us - s_last_isr_time_us) < 50000) {   /* 50ms = 50000us 消抖 */
        return;
    }
    s_last_isr_time_us = now_us;

    if (s_emergency_cb) {
        s_emergency_cb();
    }
}

esp_err_t bsp_gpio_init(void)
{
    /* === GPIO48 (LED) - 推挽输出 === */
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << APP_CFG_LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&led_conf));
    gpio_set_level(APP_CFG_LED_PIN, 0);  /* 初始熄灭 */

    /* === GPIO3 (应急按钮) - 输入上拉，下降沿中断 === */
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << APP_CFG_EMERGENCY_BTN_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn_conf));

    /* 安装 GPIO ISR 服务（ESP_ERR_INVALID_STATE 表示已安装，可忽略） */
    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "安装 ISR 服务失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 注册中断处理函数 */
    ESP_ERROR_CHECK(gpio_isr_handler_add(APP_CFG_EMERGENCY_BTN_PIN,
                                          emergency_isr_handler, NULL));

    ESP_LOGI(TAG, "GPIO 初始化完成 (LED=GPIO%d, 应急按钮=GPIO%d)",
             APP_CFG_LED_PIN, APP_CFG_EMERGENCY_BTN_PIN);
    return ESP_OK;
}

esp_err_t bsp_register_emergency_callback(emergency_btn_cb_t cb)
{
    s_emergency_cb = cb;
    return ESP_OK;
}

void bsp_led_set(bool on)
{
    gpio_set_level(APP_CFG_LED_PIN, on ? 1 : 0);
}
