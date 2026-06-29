/**
 * @file actuator_service.h
 * @brief 执行器控制服务
 *
 * 【职责】
 *   统一管理 PCA9685 的 16 路 PWM 输出，控制继电器通断。
 *   作为执行器的唯一操作入口，确保状态一致性。
 *
 * 【调用来源】
 *   - cabin_fsm（状态机切换时批量设置执行器）   → actuator_set_all()
 *   - cabin_fsm（急停/故障时全部关闭）           → actuator_all_off()
 *   - screen_service（触控手动控制灯光/空调）     → actuator_set(CH_LIGHT/CH_AC, on)
 *   - voice_service（语音指令控制灯光/空调）      → actuator_set(CH_LIGHT/CH_AC, on)
 *   - mqtt_client（远程控制）                     → actuator_set(channel, on)
 *
 * 【通道映射】（详见 cabin_fsm.h 中 CH_xxx 宏定义）
 *   CH0  = 制氧组 (AC220V)      CH8  = 泄气阀-正常开 (DC12V)
 *   CH1  = 增氧泵 (AC220V)      CH9  = 泄气阀-正常闭 (DC12V)
 *   CH2  = 进气阀-低速 (DC12V)  CH10 = 泄气阀-大流量开 (DC12V)
 *   CH3  = 进气阀-中速 (DC12V)  CH11 = 泄气阀-大流量闭 (DC12V)
 *   CH4  = 进气阀-快速 (DC12V)  CH12 = 管道排气风扇 (DC12V)
 *   CH5  = 换气阀 1~10KPa       CH13 = 灯光 (AC220V, 手动)
 *   CH6  = 换气阀 >10~20KPa     CH14 = 空调 (AC220V, 手动)
 *   CH7  = 换气阀 >20~30KPa     CH15 = 备用
 *
 * 【安全说明】
 *   - actuator_all_off() 为最高优先级安全操作，任何时候都可调用
 *   - 灯光(CH13)和空调(CH14)为手动控制，不受状态机约束
 *   - 其他通道(CH0-CH12)应只由 cabin_fsm 控制，避免冲突
 */
#ifndef ACTUATOR_SERVICE_H
#define ACTUATOR_SERVICE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化执行器服务
 *
 * 调用 pca9685_init()，并将所有通道设为关闭状态。
 * 必须在 cabin_fsm_init() 之前调用。
 *
 * @return ESP_OK 成功
 */
esp_err_t actuator_service_init(void);

/**
 * @brief 控制单个通道
 *
 * @param channel PCA9685 通道 (0-15)，使用 cabin_fsm.h 中 CH_xxx 宏
 * @param on      true=继电器吸合(供电), false=继电器断开(断电)
 * @return ESP_OK 成功，ESP_ERR_INVALID_ARG 通道号超范围
 */
esp_err_t actuator_set(uint8_t channel, bool on);

/**
 * @brief 批量设置所有通道（状态机切换时使用）
 *
 * 根据状态机表，每次状态切换时一次性更新所有执行器。
 * 例如进入"停机"状态：states = (1<<CH_DEFLATE_NORM_O) | (1<<CH_DEFLATE_NORM_C)
 *
 * @param states 16 位 bitmap，bit0=CH0...bit15=CH15，1=开 0=关
 * @return ESP_OK 成功
 */
esp_err_t actuator_set_all(uint16_t states);

/**
 * @brief 关闭所有执行器（安全操作）
 *
 * 急停、故障、应急按钮触发时调用。
 * 优先级最高，无视当前状态直接全部断电。
 *
 * @return ESP_OK 成功
 */
esp_err_t actuator_all_off(void);

/**
 * @brief 查询指定通道当前状态
 * @param channel PCA9685 通道 (0-15)
 * @return true=开启, false=关闭
 */
bool actuator_get(uint8_t channel);

/**
 * @brief 一次性获取全部 16 路通道的 on/off 位图
 *
 * 供串口屏执行器页 (Page 2) 批量刷新用 —— 每次推送时调一次 get_bitmap,
 * 和上一轮缓存 XOR 得到变化位, 只对变化的通道发屏幕指令。
 *
 * @return 16 位 bitmap, bit0=CH0 ... bit15=CH15, 1=开 0=关
 */
uint16_t actuator_get_bitmap(void);

#endif // ACTUATOR_SERVICE_H
