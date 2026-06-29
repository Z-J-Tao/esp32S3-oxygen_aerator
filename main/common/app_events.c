/**
 * @file app_events.c
 * @brief 全局事件组、队列、信号量的创建与初始化
 *
 * 【职责】
 *   定义并创建 app_events.h 中声明的全局句柄：
 *     g_system_events     — 系统事件组
 *     g_sensor_data_queue — 传感器数据队列
 *     g_rs485_mutex       — RS485 总线互斥锁
 *
 * 【调用时机】
 *   必须在 app_main 的阶段 1（基础设施）中调用，
 *   早于所有 driver / service 的初始化。
 */
#include "app_events.h"
#include "esp_log.h"

static const char *TAG __attribute__((unused)) = "app_events";

/* ==================== 全局句柄定义 ==================== */
EventGroupHandle_t g_system_events    = NULL;
QueueHandle_t      g_sensor_data_queue = NULL;
SemaphoreHandle_t  g_rs485_mutex      = NULL;

/* sensor_data_t 大小: 4 floats (16) + 1 bool (1) + padding = 20 bytes */
#define SENSOR_DATA_QUEUE_ITEM_SIZE     20
#define SENSOR_DATA_QUEUE_LENGTH        4

void app_events_init(void)
{
    /* 创建系统事件组 */
    g_system_events = xEventGroupCreate();
    assert(g_system_events != NULL);

    /* 创建传感器数据队列（长度 4，item 大小匹配 sensor_data_t）*/
    g_sensor_data_queue = xQueueCreate(SENSOR_DATA_QUEUE_LENGTH,
                                       SENSOR_DATA_QUEUE_ITEM_SIZE);
    assert(g_sensor_data_queue != NULL);

    /* 创建 RS485 互斥锁 */
    g_rs485_mutex = xSemaphoreCreateMutex();
    assert(g_rs485_mutex != NULL);

    ESP_LOGI(TAG, "事件组 / 队列 / 信号量 初始化完成");
}
