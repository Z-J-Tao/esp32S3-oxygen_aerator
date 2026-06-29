/**
 * @file bsp.c
 * @brief 板级支持包 - 统一初始化入口实现
 *
 * 【初始化顺序】（不可随意调整）
 *   1. bsp_gpio_init()          — GPIO LED、应急按钮中断
 *   2. bsp_i2c_init()           — I2C0 主机（连接 PCA9685）
 *   3. bsp_uart1_rs485_init()   — UART1 RS485（传感器 + 串口屏）
 *   4. bsp_uart2_voice_init()   — UART2（ASR PRO 语音模块）
 *
 * 【依赖】
 *   无外部依赖，但必须在 drivers 和 services 之前调用。
 */
#include "bsp.h"
#include "bsp_gpio.h"
#include "bsp_i2c.h"
#include "bsp_uart.h"
#include "esp_log.h"

static const char *TAG = "bsp";

esp_err_t bsp_init(void)
{
    esp_err_t ret;

    /* 1. GPIO — LED、应急按钮（当前为 stub，返回 ESP_OK） */
    ret = bsp_gpio_init();
    if (ret != ESP_OK) return ret;

    /* 2. I2C — PCA9685 连接（当前为 stub，返回 ESP_OK） */
    ret = bsp_i2c_init();
    if (ret != ESP_OK) return ret;

    /* 3. UART1 — RS485 总线（温湿度传感器 + 串口屏） */
    ret = bsp_uart1_rs485_init();
    if (ret != ESP_OK) return ret;

    /* 4. UART2 — ASR PRO 语音模块（当前为 stub，返回 ESP_OK） */
    ret = bsp_uart2_voice_init();
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "BSP 全部初始化完成");
    return ESP_OK;
}
