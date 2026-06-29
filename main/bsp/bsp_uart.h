/**
 * @file bsp_uart.h
 * @brief UART 外设初始化
 *
 * 本系统使用两路 UART：
 *
 * 【UART1 - RS485 总线】
 *   - 引脚：TX=GPIO17, RX=GPIO18
 *   - 波特率：9600（默认，可在 app_config.h 中修改）
 *   - 硬件：TTL 转 485 模块（内置自动 DE/RE 方向控制，无需额外方向引脚）
 *   - 用途：Modbus RTU 传感器 + 尚视界串口屏（共用总线，需 bus_scheduler 调度）
 *   - 模式：普通 UART 模式 (UART_MODE_UART)
 *
 * 【UART2 - ASR PRO 语音模块】
 *   - 引脚：TX=GPIO11, RX=GPIO12
 *   - 波特率：9600（默认）
 *   - 用途：发送播报指令 / 接收语音识别结果
 *   - 注意：全双工，无需方向控制
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include "esp_err.h"

/**
 * @brief 初始化 UART1 用于 RS485 通信
 *
 * 配置步骤：
 *   1. 设置 UART1 参数（波特率、数据位、停止位、校验）
 *   2. 分配 TX/RX 引脚（GPIO17/GPIO18）
 *   3. 安装 UART 驱动，分配 RX 环形缓冲区
 *   4. 设置普通 UART 模式 (UART_MODE_UART)
 *
 * @return ESP_OK 成功
 */
esp_err_t bsp_uart1_rs485_init(void);

/**
 * @brief 初始化 UART2 用于 ASR PRO 语音模块通信
 *
 * 配置步骤：
 *   1. 设置 UART2 参数（波特率 9600，8N1）
 *   2. 分配 TX/RX 引脚（GPIO11/GPIO12）
 *   3. 安装 UART 驱动，分配 RX 环形缓冲区
 *
 * @return ESP_OK 成功
 */
esp_err_t bsp_uart2_voice_init(void);

#endif // BSP_UART_H
