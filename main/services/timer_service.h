/**
 * @file timer_service.h
 * @brief 定时任务管理服务
 *
 * 【职责】
 *   - 管理用户设定的定时任务（如：每天 8:00 开灯，22:00 关灯）
 *   - 通过 SNTP 同步网络时间，确保定时准确
 *   - 到时间后调用 actuator_service 控制对应执行器
 *
 * 【与 cabin_fsm 的关系】
 *   timer_service 只控制手动设备（灯光 CH13、空调 CH14）的定时。
 *   气压相关的设备（CH0-CH12）由 cabin_fsm 控制，timer_service 不干预。
 *
 * 【定时任务存储】
 *   保存在 NVS 中，掉电不丢失。
 *   通过串口屏或 MQTT 添加/修改/删除。
 *
 * 【SNTP 时间同步】
 *   系统启动并连接 WiFi 后，自动同步 NTP 服务器时间。
 *   同步成功后定时任务才开始生效。
 */
#ifndef TIMER_SERVICE_H
#define TIMER_SERVICE_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief 查询 SNTP 时间是否已同步
 *
 * 同步前调用 time() 返回值不可信（从 1970 开始计），
 * 显示模块应据此显示占位符（如 "--:--"）。
 *
 * @return true=已同步, false=未同步
 */
bool timer_service_is_time_synced(void);

/**
 * @brief 初始化定时任务服务
 *
 * 从 NVS 加载已保存的定时任务列表。
 *
 * @return ESP_OK 成功
 */
esp_err_t timer_service_init(void);

/**
 * @brief 启动定时任务服务
 *
 * 创建 FreeRTOS 任务 "timer_task"：
 *   1. 初始化 SNTP，同步网络时间
 *   2. 每分钟检查一次定时任务列表
 *   3. 匹配当前时间，触发对应操作
 *
 * @return ESP_OK 任务创建成功
 */
esp_err_t timer_service_start(void);

#endif // TIMER_SERVICE_H
