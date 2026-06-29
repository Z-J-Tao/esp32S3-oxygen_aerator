/**
 * @file bsp.h
 * @brief 板级支持包 - 统一初始化入口
 *
 * 调用此函数会依次初始化所有板级外设：
 *   1. bsp_gpio_init()  - GPIO（LED、应急按钮）
 *   2. bsp_i2c_init()   - I2C 总线（连接 PCA9685）
 *   3. bsp_uart1_rs485_init() - UART1 RS485（传感器 + 串口屏）
 *   4. bsp_uart2_voice_init() - UART2（ASR PRO 语音模块）
 *
 * 必须在 drivers 和 services 初始化之前调用。
 */
#ifndef BSP_H
#define BSP_H

#include "esp_err.h"

/**
 * @brief 板级统一初始化，依次初始化 GPIO → I2C → UART1 → UART2
 * @return ESP_OK 成功，其他值表示某个外设初始化失败
 */
esp_err_t bsp_init(void);

#endif // BSP_H
