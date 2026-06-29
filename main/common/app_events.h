/**
 * @file app_events.h
 * @brief 全局事件组、队列、信号量定义
 *
 * 【设计说明】
 *   本系统基于 FreeRTOS 的事件组实现模块间松耦合通信。
 *   各服务通过设置/等待事件位来协调工作，避免直接函数调用带来的耦合。
 *
 * 【事件位分配】
 *   BIT0: EVT_WIFI_CONNECTED     — WiFi 已连接并获得 IP
 *   BIT1: EVT_WIFI_DISCONNECTED  — WiFi 断线
 *   BIT2: EVT_MQTT_CONNECTED     — MQTT 已连接 broker
 *   BIT3: EVT_MQTT_DISCONNECTED  — MQTT 断线
 *   BIT4: EVT_SENSOR_DATA_READY  — 传感器新一轮数据采集完成
 *   BIT5: EVT_ALARM_TRIGGERED    — 规则引擎触发了报警
 *   BIT6: EVT_EMERGENCY          — 应急按钮被按下
 *
 * 【全局句柄】
 *   g_system_events:     事件组，所有模块共用
 *   g_sensor_data_queue: 传感器数据队列（sensor → rule_engine / screen / mqtt）
 *   g_rs485_mutex:       RS485 总线互斥锁（sensor_service 与 screen_service 共用）
 */
#ifndef APP_EVENTS_H
#define APP_EVENTS_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* ==================== 事件组位定义 ==================== */

#define EVT_WIFI_CONNECTED    BIT0    // wifi_manager: 已连接
#define EVT_WIFI_DISCONNECTED BIT1    // wifi_manager: 断线
#define EVT_MQTT_CONNECTED    BIT2    // mqtt_client: 已连接
#define EVT_MQTT_DISCONNECTED BIT3    // mqtt_client: 断线
#define EVT_SENSOR_DATA_READY BIT4    // sensor_service: 新数据就绪
#define EVT_ALARM_TRIGGERED   BIT5    // rule_engine: 报警触发
#define EVT_EMERGENCY         BIT6    // bsp_gpio: 应急按钮按下

/* ==================== 全局句柄声明 ==================== */

/** 系统事件组 — 所有模块通过此事件组通信 */
extern EventGroupHandle_t g_system_events;

/** 传感器数据队列 — sensor_service 生产，rule_engine/screen/mqtt 消费 */
extern QueueHandle_t      g_sensor_data_queue;

/** RS485 总线互斥锁 — sensor_service 和 screen_service 竞争使用 */
extern SemaphoreHandle_t  g_rs485_mutex;

/**
 * @brief 初始化所有全局事件组、队列、信号量
 *
 * 必须在所有 service 初始化之前调用（在 app_main 的第一步）。
 */
void app_events_init(void);

#endif // APP_EVENTS_H
