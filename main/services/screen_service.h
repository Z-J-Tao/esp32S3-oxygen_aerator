/**
 * @file screen_service.h
 * @brief 串口屏业务服务
 *
 * 【职责】
 *   - 定时将传感器数据、氧舱状态推送到串口屏显示 (使用 set_text)
 *   - 接收用户在串口屏上的触控操作，分发给对应模块 (cabin_fsm/actuator)
 *   - 管理 RS485 总线访问 (通过 bus_scheduler)
 *
 * 【与 screen_protocol 的关系】
 *   screen_protocol（驱动层）：负责帧的封装/解析/CRC、UART 收发
 *   screen_service （服务层）：负责业务逻辑，决定"推什么数据"和"触控了做什么"
 *
 * 【任务结构】
 *   screen_tx_task: 定时推送数据到屏幕 (优先级4)
 *   screen_rx_task: 持续接收 UART 数据喂给协议解析器 (优先级4)
 */
#ifndef SCREEN_SERVICE_H
#define SCREEN_SERVICE_H

#include "esp_err.h"

/**
 * @brief 启动串口屏服务
 *
 * 初始化协议层，注册触控回调，创建收发任务。
 *
 * @return ESP_OK 成功
 */
esp_err_t screen_service_start(void);

#endif /* SCREEN_SERVICE_H */
