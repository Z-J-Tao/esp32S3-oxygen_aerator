/**
 * @file bsp_gpio.h
 * @brief 通用 GPIO 初始化（调试LED、应急按钮）
 *
 * 本文件管理的 GPIO：
 *
 * 【GPIO48 - 调试 LED】
 *   - 输出模式，推挽
 *   - 高电平点亮，低电平熄灭
 *   - 用于系统状态指示（如：运行中闪烁、故障常亮等）
 *
 * 【GPIO3 - 应急按钮】
 *   - 输入模式，内部上拉
 *   - 按下时为低电平（下降沿触发中断）
 *   - 连接到外部应急按钮 (ZX-QC66-8.5CJ)
 *   - 触发后应立即进入应急状态：关闭所有气阀、停止升压
 *   - 需要做软件消抖处理（建议 50ms）
 */
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief 初始化所有通用 GPIO
 *
 * 配置：
 *   - GPIO48 (LED):      推挽输出，初始低电平（灭）
 *   - GPIO3  (应急按钮):  输入上拉，下降沿中断，注册 ISR
 *
 * @return ESP_OK 成功
 */
esp_err_t bsp_gpio_init(void);

/**
 * @brief 应急按钮中断回调函数类型
 *
 * 注意：此回调在 ISR 上下文中被调用（通过 gpio_isr_handler），
 * 不能执行耗时操作，建议只做 xEventGroupSetBitsFromISR() 或
 * xQueueSendFromISR() 等 ISR 安全操作。
 */
typedef void (*emergency_btn_cb_t)(void);

/**
 * @brief 注册应急按钮触发回调
 * @param cb 回调函数指针，传 NULL 可取消注册
 * @return ESP_OK 成功
 */
esp_err_t bsp_register_emergency_callback(emergency_btn_cb_t cb);

/**
 * @brief 设置调试 LED 状态
 * @param on true=点亮, false=熄灭
 */
void bsp_led_set(bool on);

#endif // BSP_GPIO_H
