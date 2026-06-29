/**
 * @file voice_module.c
 * @brief ASR PRO 离线语音模块驱动实现
 *
 * 通信协议：单字节十六进制，UART2 9600 8N1，全双工点对点
 * 无帧头、帧尾、校验 —— 专用 UART 不存在总线冲突，1 字节足够。
 *
 * 【接收识别结果】ASR PRO → ESP32（1 字节）
 *   0x01=启动  0x02=暂停  0x03=继续  0x04=停止  0x05=紧急停止
 *   0x10=开灯  0x11=关灯  0x12=开空调  0x13=关空调
 *   0x20=换气  0x21=消毒
 *
 * 【发送播报指令】ESP32 → ASR PRO（1 字节）
 *   0x01=系统启动  0x02=开始升压  0x03=保压运行中  0x04=开始泄压
 *   0x05=本次使用结束  0x10=温度异常  0x11=压力异常  0x12=氧浓度异常
 *   0x20=紧急停机
 */
#include "voice_module.h"
#include "common/app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

static const char *TAG = "voice_module";

/* 语音识别回调 */
static voice_recognize_cb_t s_recognize_cb = NULL;

/* 接收任务句柄，便于后续 suspend/delete */
static TaskHandle_t s_rx_task_handle = NULL;

/**
 * @brief UART2 接收任务
 *
 * 循环读取 1 字节，uart_read_bytes 本身阻塞等待，无需额外 vTaskDelay。
 * 收到数据后通过回调通知 voice_service。
 */
static void voice_rx_task(void *arg)
{
    uint8_t byte;
    while (1) {
        /* 阻塞等待 100ms，无数据则自动进入下一轮，不额外 delay */
        int len = uart_read_bytes(APP_CFG_VOICE_UART_NUM, &byte, 1, pdMS_TO_TICKS(100));
        if (len == 1) {
            ESP_LOGI(TAG, "收到语音指令: 0x%02X", byte);
            if (s_recognize_cb) {
                s_recognize_cb(byte);
            }
        }
    }
}

esp_err_t voice_module_init(void)
{
    /* 创建 UART2 接收任务，优先级 3（voice/timer 层）
     * 栈 3072: rx 回调可能进入 voice_service 的 on_voice_command,
     * 内部走 cabin_fsm_send_event/actuator_set + 多次 ESP_LOGI, 2048 偏紧 */
    BaseType_t ret = xTaskCreate(voice_rx_task, "voice_rx", 3072, NULL, 3, &s_rx_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 voice_rx 任务失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ASR PRO 语音模块初始化完成");
    return ESP_OK;
}

esp_err_t voice_play(uint8_t voice_id)
{
    /* 发送 1 字节播报指令到 ASR PRO */
    int len = uart_write_bytes(APP_CFG_VOICE_UART_NUM, &voice_id, 1);
    if (len > 0) {
        ESP_LOGI(TAG, "发送播报指令: 0x%02X", voice_id);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "发送播报指令失败");
        return ESP_FAIL;
    }
}

esp_err_t voice_register_recognize_callback(voice_recognize_cb_t cb)
{
    s_recognize_cb = cb;
    return ESP_OK;
}
