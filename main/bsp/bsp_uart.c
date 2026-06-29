/**
 * @file bsp_uart.c
 * @brief UART 外设初始化实现
 *
 * 引脚定义参见 common/app_config.h。
 * UART1: RS485 总线（TTL转485模块，自动方向控制）
 * UART2: ASR PRO 语音模块
 */
#include "bsp_uart.h"
#include "common/app_config.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "bsp_uart";

/* UART1 缓冲区大小 */
#define RS485_RX_BUF_SIZE   512
#define RS485_TX_BUF_SIZE   256

/* UART2 缓冲区大小 */
#define VOICE_RX_BUF_SIZE   512
#define VOICE_TX_BUF_SIZE   256

esp_err_t bsp_uart1_rs485_init(void)
{
    /* 步骤 1: 配置 UART 参数 — 9600 8N1，与温湿度传感器手册一致 */
    uart_config_t uart_config = {
        .baud_rate  = APP_CFG_RS485_BAUD_RATE,   // 9600
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(APP_CFG_RS485_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART1 参数配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 步骤 2: 设置引脚 — TX=GPIO17, RX=GPIO18, 无 RTS/CTS */
    ret = uart_set_pin(APP_CFG_RS485_UART_NUM,
                       APP_CFG_RS485_TX_PIN,     // GPIO17
                       APP_CFG_RS485_RX_PIN,     // GPIO18
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART1 引脚配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 步骤 3: 安装驱动 — RX 512B, TX 256B, 不使用事件队列 */
    ret = uart_driver_install(APP_CFG_RS485_UART_NUM,
                              RS485_RX_BUF_SIZE,
                              RS485_TX_BUF_SIZE,
                              0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART1 驱动安装失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 步骤 4: 普通 UART 模式（TTL转485模块自动控制方向） */
    ret = uart_set_mode(APP_CFG_RS485_UART_NUM, UART_MODE_UART);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART1 模式设置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART1 RS485 初始化完成 (9600 8N1, TX:GPIO%d, RX:GPIO%d)",
             APP_CFG_RS485_TX_PIN, APP_CFG_RS485_RX_PIN);
    return ESP_OK;
}

esp_err_t bsp_uart2_voice_init(void)
{
    /* 步骤 1: 配置 UART 参数 — 9600 8N1，与 ASR PRO 语音模块一致 */
    uart_config_t uart_config = {
        .baud_rate  = APP_CFG_VOICE_BAUD_RATE,   // ASR PRO 默认波特率
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(APP_CFG_VOICE_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART2 参数配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 步骤 2: 设置引脚 — TX=GPIO11, RX=GPIO12, 无 RTS/CTS */
    ret = uart_set_pin(APP_CFG_VOICE_UART_NUM,
                       APP_CFG_VOICE_TX_PIN,     // TX=GPIO11
                       APP_CFG_VOICE_RX_PIN,     // RX=GPIO12
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART2 引脚配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 步骤 3: 安装驱动 — RX 512B, TX 256B, 不使用事件队列 */
    ret = uart_driver_install(APP_CFG_VOICE_UART_NUM,
                              VOICE_RX_BUF_SIZE,
                              VOICE_TX_BUF_SIZE,
                              0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART2 驱动安装失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART2 ASR PRO 初始化完成 (9600 8N1, TX:GPIO%d, RX:GPIO%d)",
             APP_CFG_VOICE_TX_PIN, APP_CFG_VOICE_RX_PIN);
    return ESP_OK;
}
