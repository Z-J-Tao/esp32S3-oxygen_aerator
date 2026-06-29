/**
 * @file bus_scheduler.h
 * @brief RS485 总线分时调度器
 *
 * 【问题背景】
 *   Modbus 传感器和尚视界串口屏共用同一路 RS485 总线（UART1），
 *   RS485 是半双工总线，同一时刻只能有一个设备在通信。
 *   如果传感器轮询和屏幕刷新同时操作 UART1，数据会互相干扰。
 *
 * 【解决方案】
 *   使用 FreeRTOS 互斥锁（Mutex）实现总线的排他访问：
 *   - sensor_service 轮询前调用 bus_acquire(BUS_USER_SENSOR, timeout)
 *   - screen_service 刷新前调用 bus_acquire(BUS_USER_SCREEN, timeout)
 *   - 操作完成后调用 bus_release() 释放
 *
 * 【调度策略】
 *   使用简单的互斥锁即可，FreeRTOS 会自动处理优先级继承。
 *   建议 sensor_service 的任务优先级高于 screen_service，
 *   这样传感器采集会优先于屏幕刷新（传感器数据更实时敏感）。
 *
 * 【时间分配参考】
 *   - 传感器轮询：每 2 秒一轮，每轮占用 ~200ms（3个传感器 × ~60ms）
 *   - 屏幕刷新：每 500ms 一次，每次占用 ~50ms
 *   - 总线利用率：(200+100)/2000 = 15%，绰绰有余
 */
#ifndef BUS_SCHEDULER_H
#define BUS_SCHEDULER_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief 总线使用者标识
 */
typedef enum {
    BUS_USER_SENSOR,    // Modbus 传感器轮询
    BUS_USER_SCREEN,    // 尚视界串口屏通信
} bus_user_t;

/**
 * @brief 初始化总线调度器
 *
 * 创建互斥锁 g_rs485_mutex。
 *
 * @return ESP_OK 成功
 */
esp_err_t bus_scheduler_init(void);

/**
 * @brief 请求占用 485 总线
 *
 * 阻塞等待直到获取互斥锁，或超时返回。
 *
 * @param user       请求者标识（用于日志和调试）
 * @param timeout_ms 最大等待时间（ms），0=不等待，portMAX_DELAY=永久等待
 * @return ESP_OK 成功获取，ESP_ERR_TIMEOUT 超时
 */
esp_err_t bus_acquire(bus_user_t user, uint32_t timeout_ms);

/**
 * @brief 释放 485 总线
 *
 * @param user 释放者标识（应与 acquire 时一致）
 * @return ESP_OK 成功
 */
esp_err_t bus_release(bus_user_t user);

#endif // BUS_SCHEDULER_H
