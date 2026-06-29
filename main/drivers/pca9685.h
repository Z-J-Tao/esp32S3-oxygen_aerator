/**
 * @file pca9685.h
 * @brief PCA9685PW I2C 16路 PWM 控制器驱动
 *
 * 【芯片说明】
 *   PCA9685PW,118 是 NXP 的 16 通道 12-bit PWM 控制器，通过 I2C 控制。
 *   每个通道可独立设置 PWM 占空比（0-4095）。
 *
 * 【硬件连接】
 *   - I2C 地址: 0x40（A0-A5 全接地）
 *   - SDA: GPIO1, SCL: GPIO2（经 3.3k 上拉）
 *   - OE# (Output Enable): 低电平使能（原理图中接地，始终使能）
 *   - EXTCLK: 未使用，使用内部 25MHz 振荡器
 *
 * 【本系统中的用法】
 *   16 路 PWM 输出连接继电器模块，用于控制氧舱设备：
 *   - 继电器只有通断两种状态：全开(on=0, off=4095) 或 全关(on=0, off=0)
 *   - 通道 0-14: 对应 15 个执行器（见 cabin_fsm.h 中 CH_xxx 定义）
 *   - 通道 15: 备用
 *   - 每个 PWM 输出连接一个 LED 指示灯用于测试
 *
 * 【关键寄存器】
 *   - MODE1 (0x00): 模式寄存器1（SLEEP, RESTART, AI 等）
 *   - PRE_SCALE (0xFE): PWM 频率预分频器（需在 SLEEP 模式下设置）
 *   - LED0_ON_L  (0x06): 通道0 ON 低字节，后续每通道偏移 4 字节
 *   - LED0_ON_H  (0x07): 通道0 ON 高字节
 *   - LED0_OFF_L (0x08): 通道0 OFF 低字节
 *   - LED0_OFF_H (0x09): 通道0 OFF 高字节
 */
#ifndef PCA9685_H
#define PCA9685_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief 初始化 PCA9685
 *
 * 初始化步骤：
 *   1. 发送 SLEEP 模式命令（MODE1 寄存器 bit4=1）
 *   2. 设置 PRE_SCALE 寄存器（PWM 频率，继电器控制建议 50-200Hz）
 *      PRE_SCALE = round(25MHz / (4096 * freq)) - 1
 *      例如 50Hz: PRE_SCALE = round(25000000 / (4096 * 50)) - 1 = 121
 *   3. 退出 SLEEP，设置 RESTART 和 AI（自动递增）
 *   4. 关闭所有通道输出
 *
 * @return ESP_OK 成功，ESP_FAIL I2C 通信失败
 */
esp_err_t pca9685_init(void);

/**
 * @brief 设置指定通道的 PWM 占空比
 *
 * PCA9685 每个 PWM 周期有 4096 个计数 (0-4095)：
 *   - on:  计数到 on 时输出变高
 *   - off: 计数到 off 时输出变低
 *   - 占空比 = (off - on) / 4096
 *
 * @param channel 通道号 (0-15)
 * @param on      高电平起始点 (0-4095)，通常设为 0
 * @param off     高电平结束点 (0-4095)，4095=全开，0=全关
 * @return ESP_OK 成功
 */
esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off);

/**
 * @brief 通道全开（继电器吸合/设备供电）
 *
 * 等效于 pca9685_set_pwm(channel, 0, 4095)
 * 利用 LED_ON_H 的 bit4（full-on bit）实现硬件全开
 *
 * @param channel 通道号 (0-15)
 * @return ESP_OK 成功
 */
esp_err_t pca9685_channel_on(uint8_t channel);

/**
 * @brief 通道全关（继电器断开/设备断电）
 *
 * 等效于 pca9685_set_pwm(channel, 0, 0)
 * 利用 LED_OFF_H 的 bit4（full-off bit）实现硬件全关
 *
 * @param channel 通道号 (0-15)
 * @return ESP_OK 成功
 */
esp_err_t pca9685_channel_off(uint8_t channel);

/**
 * @brief 所有通道全关（一条 I2C 指令）
 *
 * 使用 ALL_LED_OFF_H 寄存器 bit4 一次性关闭全部 16 通道，
 * 比逐通道关闭更快，适用于急停场景。
 *
 * @return ESP_OK 成功
 */
esp_err_t pca9685_all_off(void);

#endif // PCA9685_H
