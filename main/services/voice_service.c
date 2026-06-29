/**
 * @file voice_service.c
 * @brief 语音交互业务服务实现
 *
 * 单字节协议：ASR PRO 识别到语音后发 1 字节 cmd_id 到 UART2，
 * voice_module 接收后回调本模块的 on_voice_command() 进行业务分发。
 */
#include "voice_service.h"
#include "drivers/voice_module.h"
#include "services/cabin_fsm.h"
#include "services/actuator_service.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "voice_service";

/* 播报队列 */
static QueueHandle_t s_play_queue = NULL;

/* 播报任务句柄 */
static TaskHandle_t s_play_task_handle = NULL;

/**
 * @brief 语音指令处理回调（在 voice_rx_task 上下文中执行）
 *
 * cabin_fsm_send_event / actuator_set 内部通过队列投递，不会阻塞。
 * 如果后续这些函数变为阻塞型，需改用队列转发到独立任务处理。
 */
static void on_voice_command(uint8_t cmd_id)
{
    ESP_LOGI(TAG, "收到语音指令: 0x%02X", cmd_id);

    switch (cmd_id) {
        /* ── 状态机控制 ── */
        case 0x01:  /* "启动" */
            cabin_fsm_send_event(FSM_EVT_START);
            break;
        case 0x02:  /* "暂停" */
            cabin_fsm_send_event(FSM_EVT_PAUSE);
            break;
        case 0x03:  /* "继续" */
            cabin_fsm_send_event(FSM_EVT_RESUME);
            break;
        case 0x04:  /* "停止" */
            cabin_fsm_send_event(FSM_EVT_RESET);
            break;
        case 0x05:  /* "紧急停止" */
            cabin_fsm_send_event(FSM_EVT_ESTOP);
            break;

        /* ── 设备控制 ── */
        case 0x10:  /* "开灯" */
            actuator_set(CH_LIGHT, true);
            break;
        case 0x11:  /* "关灯" */
            actuator_set(CH_LIGHT, false);
            break;
        case 0x12:  /* "开空调" */
            actuator_set(CH_AC, true);
            break;
        case 0x13:  /* "关空调" */
            actuator_set(CH_AC, false);
            break;

        /* ── 扩展功能 ── */
        case 0x20:  /* "换气" */
            cabin_fsm_send_event(FSM_EVT_VENTILATE);
            break;
        case 0x21:  /* "消毒" */
            cabin_fsm_send_event(FSM_EVT_DISINFECT);
            break;

        default:
            ESP_LOGW(TAG, "未知语音指令: 0x%02X", cmd_id);
            break;
    }
}

/**
 * @brief 语音播报任务
 *
 * 从队列依次取出 voice_id 并发送到 ASR PRO，
 * 每次播报后等待 2s，确保当前语音播放完毕再发下一条。
 */
static void voice_play_task(void *arg)
{
    uint8_t voice_id;
    while (1) {
        if (xQueueReceive(s_play_queue, &voice_id, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "播放语音: 0x%02X", voice_id);
            voice_play(voice_id);
            vTaskDelay(pdMS_TO_TICKS(2000));    /* 等待 ASR PRO 播放完毕 */
        }
    }
}

esp_err_t voice_service_start(void)
{
    /* 注册语音识别回调 */
    voice_register_recognize_callback(on_voice_command);

    /* 创建播报队列（深度 8），防止并发播报冲突 */
    s_play_queue = xQueueCreate(8, sizeof(uint8_t));
    if (s_play_queue == NULL) {
        ESP_LOGE(TAG, "创建播报队列失败");
        return ESP_FAIL;
    }

    /* 创建语音播报任务，优先级 3（voice/timer 层）
     * 栈 4096: ESP_LOGI 单次约 1.5KB, voice_play_task → voice_play 嵌套两层 LOG,
     * 2048 实测溢出 (启动按钮触发 PRESSURIZING_INIT 的语音播报时崩溃) */
    BaseType_t ret = xTaskCreate(voice_play_task, "voice_play", 4096, NULL, 3, &s_play_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 voice_play 任务失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "语音服务启动完成");
    return ESP_OK;
}

esp_err_t voice_service_play(uint8_t voice_id)
{
    /* 通过队列串行化播报，避免冲突 */
    if (xQueueSend(s_play_queue, &voice_id, pdMS_TO_TICKS(100)) == pdTRUE) {
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "播报队列满，丢弃 voice_id=0x%02X", voice_id);
        return ESP_FAIL;
    }
}
