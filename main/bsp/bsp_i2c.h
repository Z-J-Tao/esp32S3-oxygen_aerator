/**
 * @file bsp_i2c.h
 * @brief I2C 总线初始化
 *
 * 本系统使用 I2C0 总线连接 PCA9685PW（16路 PWM 控制器）：
 *   - SDA = GPIO1
 *   - SCL = GPIO2
 *   - 频率 = 100kHz
 *   - 外部上拉：R12=3.3k (SDA), R13=3.3k (SCL)（已在 PCB 上焊接）
 *
 * PCA9685 I2C 地址：0x40（A0-A5 全接地）
 */
#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "esp_err.h"

/**
 * @brief 初始化 I2C0 主机总线
 *
 * 配置步骤：
 *   1. 设置 I2C 主机参数（SDA/SCL 引脚、时钟频率）
 *   2. 安装 I2C 驱动
 *
 * @return ESP_OK 成功
 */
esp_err_t bsp_i2c_init(void);

#endif // BSP_I2C_H
