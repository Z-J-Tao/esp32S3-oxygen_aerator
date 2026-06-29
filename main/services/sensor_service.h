/**
 * @file sensor_service.h
 * @brief 传感器采集服务
 *
 * 【职责】
 *   - 创建 FreeRTOS 任务，按 APP_CFG_MODBUS_POLL_MS 间隔轮询所有 Modbus 传感器
 *   - 通过 bus_scheduler 获取 485 总线后，调用 modbus_master 读取数据
 *   - 将采集数据存入全局缓存，并通过事件通知其他服务（串口屏刷新、规则引擎判断等）
 *
 * 【数据流向】
 *   sensor_service → g_sensor_data_queue → rule_engine（阈值判断）
 *   sensor_service → screen_service（显示刷新）
 *   sensor_service → mqtt_client（数据上报）
 *   sensor_service → cabin_fsm（压力值 → FSM_EVT_PRESSURE_UPDATE）
 *
 * 【当前传感器】
 *   - 温湿度传感器（Modbus 从站地址 0x01，功能码 0x03）
 *   - 气压传感器（Modbus 从站地址 0x01，功能码 0x03，波特率 9600，无校验位）
 *     采用整数法读取寄存器 0x0002~0x0004：
 *       0x0002 = 单位编号（1=KPa, 0=MPa, 2=Pa 等）
 *       0x0003 = 小数位数（0~4 位）
 *       0x0004 = 测量值（int16，有符号）
 *     换算公式: pressure = measurement / 10^decimal_places
 *   - 氧气浓度传感器（通过 RP-485 转接板，Modbus 从站地址 0x16，功能码 0x04）
 *     寄存器 0x0068 = 氧浓度 (uint16, /10.0 → %)
 *     寄存器 0x0069 = 氧流量 (uint16, /10.0 → L/min)
 *   - 更多传感器可通过 app_config.h 中的 Modbus 地址表扩展
 *
 * 【注意】
 *   轮询任务优先级应适中（建议 5），不能阻塞其他关键任务
 *   485 总线是共享资源，采集完一轮后必须及时释放
 */
#ifndef SENSOR_SERVICE_H
#define SENSOR_SERVICE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 传感器数据结构（一次轮询的完整数据）
 *
 * 所有传感器的最新值都存在这里，供其他服务读取。
 * 各字段的单位和精度取决于具体传感器的 Modbus 寄存器定义。
 */
typedef struct {
    float temperature;          // 温度 (℃)
    float humidity;             // 湿度 (%)
    float pressure_kpa;         // 氧舱内压力 (KPa) — 状态机核心数据
    float oxygen_percent;       // 氧气浓度 (%) — OCS-3F3.0 经 RP-485 转接板读取
    float oxygen_flow;          // 氧气流量 (L/min) — OCS-3F3.0 经 RP-485 转接板读取
    bool  valid;                // 数据是否有效（至少成功采集过一次）
} sensor_data_t;

/**
 * @brief 启动传感器采集服务
 *
 * 创建 FreeRTOS 任务 "sensor_task"，循环执行：
 *   1. bus_acquire(BUS_USER_SENSOR, timeout) 获取 485 总线
 *   2. 依次调用 modbus_master_read_holding() / modbus_master_read_input() 读取各传感器
 *   3. bus_release(BUS_USER_SENSOR) 释放总线
 *   4. 更新 sensor_data_t 缓存
 *   5. 设置 EVT_SENSOR_DATA_READY 事件位
 *   6. 向 cabin_fsm 发送 FSM_EVT_PRESSURE_UPDATE（如果压力值有变化）
 *   7. vTaskDelay(APP_CFG_MODBUS_POLL_MS) 等待下一轮
 *
 * @return ESP_OK 任务创建成功
 */
esp_err_t sensor_service_start(void);

/**
 * @brief 获取最新的传感器数据（线程安全）
 * @param data [out] 拷贝最新数据到调用者提供的结构体
 * @return ESP_OK 成功，ESP_ERR_INVALID_STATE 数据尚未有效
 */
esp_err_t sensor_service_get_data(sensor_data_t *data);

#endif // SENSOR_SERVICE_H
