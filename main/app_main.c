/**
 * @file app_main.c
 * @brief 微压增氧控制系统 — 应用入口
 *
 * 【系统初始化顺序】（顺序不能随意调整！）
 *
 *   阶段 1: 基础设施
 *     data_store_init()    — NVS 存储（后续模块需要读配置）
 *     app_events_init()    — 事件组/队列/信号量（后续模块需要使用）
 *     bsp_init()           — 硬件外设 GPIO/I2C/UART
 *
 *   阶段 2: 设备驱动
 *     modbus_master_init() — Modbus 主站（依赖 UART1）
 *     pca9685_init()       — PCA9685 PWM（依赖 I2C）
 *     screen_protocol_init() — 串口屏协议（依赖 UART1）
 *     voice_module_init()  — ASR PRO 语音（依赖 UART2）
 *
 *   阶段 3: 业务服务
 *     bus_scheduler_init() — 485 总线调度器（创建互斥锁）
 *     actuator_service_init() — 执行器服务（依赖 PCA9685，全部关闭）
 *     cabin_fsm_init()     — 氧舱状态机（依赖 actuator_service）
 *     rule_engine_init()   — 规则引擎（从 NVS 加载规则）
 *     timer_service_init() — 定时任务（从 NVS 加载任务）
 *
 *   阶段 4: 网络
 *     wifi_manager_init()  — WiFi 连接
 *     mqtt_client_init()   — MQTT 客户端（依赖 WiFi）
 *
 *   阶段 5: 启动任务
 *     sensor_service_start()  — 传感器轮询任务
 *     screen_service_start()  — 串口屏服务任务
 *     voice_service_start()   — 语音服务
 *     cabin_fsm_start()       — 状态机任务
 *     rule_engine_start()     — 规则引擎任务
 *     timer_service_start()   — 定时任务
 *
 * 【FreeRTOS 任务优先级建议】
 *   优先级 6: cabin_fsm_task   — 状态机（最高，安全关键）
 *   优先级 5: sensor_task      — 传感器采集
 *   优先级 4: screen_task      — 串口屏刷新
 *   优先级 4: rule_task        — 规则引擎
 *   优先级 3: voice_play_task  — 语音播报
 *   优先级 3: timer_task       — 定时任务
 */
#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"

#include "common/app_config.h"
#include "common/app_events.h"
#include "common/data_store.h"
#include "bsp/bsp.h"
#include "drivers/modbus_master.h"
#include "drivers/pca9685.h"
#include "drivers/screen_protocol.h"
#include "drivers/voice_module.h"
#include "services/bus_scheduler.h"
#include "services/sensor_service.h"
#include "services/actuator_service.h"
#include "services/screen_service.h"
#include "services/voice_service.h"
#include "services/cabin_fsm.h"
#include "services/rule_engine.h"
#include "services/timer_service.h"
#include "network/wifi_manager.h"
#include "network/app_mqtt.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "=== 微压增氧控制系统启动 ===");

    /* ---- 阶段 1: 基础设施 ---- */
    data_store_init();      // NVS 存储
    app_events_init();      // 事件组 / 队列 / 信号量
    bsp_init();             // GPIO / I2C / UART 硬件初始化

    /* ---- 阶段 2: 设备驱动 ---- */
    modbus_master_init();   // Modbus RTU 主站 (UART1 RS485)
    pca9685_init();         // PCA9685 16路 PWM (I2C)
    screen_protocol_init(); // 尚视界串口屏协议 (UART1 RS485)
    voice_module_init();    // ASR PRO 语音模块 (UART2)

    /* ---- 阶段 3: 业务服务初始化 ---- */
    bus_scheduler_init();       // 485 总线分时调度器
    actuator_service_init();    // 执行器服务（上电全关）
    cabin_fsm_init();           // 氧舱状态机（初始: SHUTDOWN）
    rule_engine_init();         // 规则引擎（加载报警阈值）
    timer_service_init();       // 定时任务（加载定时表）

    /* ---- 阶段 4: 网络 ---- */
    wifi_manager_init();    // WiFi Station 连接
    mqtt_client_init();     // MQTT 客户端

    /* ---- 阶段 5: 启动 FreeRTOS 任务 ---- */
    sensor_service_start();     // 传感器轮询 (优先级 5)
    screen_service_start();     // 串口屏服务 (优先级 4)
    voice_service_start();      // 语音服务   (优先级 3)
    cabin_fsm_start();          // 状态机     (优先级 6)
    rule_engine_start();        // 规则引擎   (优先级 4)
    timer_service_start();      // 定时任务   (优先级 3)

    ESP_LOGI(TAG, "=== 所有服务已启动 ===");
}
