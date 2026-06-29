/**
 * @file actuator_service.c
 * @brief 执行器控制服务实现
 *
 * 通过 PCA9685 的 16 路 PWM 输出控制继电器通断。
 * 维护一个 16 位 bitmap 记录当前各通道状态，
 * actuator_set_all() 做差量更新，只操作有变化的通道。
 */
#include "actuator_service.h"
#include "drivers/pca9685.h"
#include "common/app_config.h"
#include "esp_log.h"

static const char *TAG = "actuator";

/* 16 路执行器状态 bitmap: bit0=CH0 ... bit15=CH15, 1=开 0=关 */
static uint16_t s_actuator_states = 0;

/* 通道名称，用于日志 */
static const char *CH_NAMES[16] = {
    "制氧组",     "增氧泵",     "进气阀低速", "进气阀中速",
    "进气阀快速", "换气阀10K",  "换气阀20K",  "换气阀30K",
    "泄气正常开", "泄气正常关", "泄气大流开", "泄气大流关",
    "排气风扇",   "灯光",       "空调",       "备用",
};

esp_err_t actuator_service_init(void)
{
    /* 初始化 PCA9685 芯片 (内部已全关所有通道) */
    esp_err_t ret = pca9685_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }

    s_actuator_states = 0;

    ESP_LOGI(TAG, "执行器服务初始化完成 (15路继电器，全部关闭)");
    return ESP_OK;
}

esp_err_t actuator_set(uint8_t channel, bool on)
{
    if (channel > 15) {
        ESP_LOGE(TAG, "通道号超范围: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    /* 状态未变则跳过 */
    bool cur = (s_actuator_states >> channel) & 0x01;
    if (cur == on) {
        return ESP_OK;
    }

    esp_err_t ret;
    if (on) {
        ret = pca9685_channel_on(channel);
        s_actuator_states |= (1 << channel);
    } else {
        ret = pca9685_channel_off(channel);
        s_actuator_states &= ~(1 << channel);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CH%d [%s] %s 失败", channel, CH_NAMES[channel], on ? "开" : "关");
    } else {
        ESP_LOGI(TAG, "CH%d [%s] → %s", channel, CH_NAMES[channel], on ? "开" : "关");
    }

    return ret;
}

esp_err_t actuator_set_all(uint16_t states)
{
    uint16_t diff = s_actuator_states ^ states;

    /* 只操作有变化的通道 */
    for (int i = 0; i < 16; i++) {
        if (diff & (1 << i)) {
            bool on = (states >> i) & 0x01;
            esp_err_t ret;
            if (on) {
                ret = pca9685_channel_on(i);
            } else {
                ret = pca9685_channel_off(i);
            }
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "CH%d [%s] 批量设置失败", i, CH_NAMES[i]);
            }
        }
    }

    ESP_LOGI(TAG, "执行器批量更新: 0x%04X → 0x%04X (变化: 0x%04X)",
             s_actuator_states, states, diff);
    s_actuator_states = states;
    return ESP_OK;
}

esp_err_t actuator_all_off(void)
{
    /* 一条 I2C 指令全关 16 通道 */
    esp_err_t ret = pca9685_all_off();

    s_actuator_states = 0;

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "紧急全关失败");
    } else {
        ESP_LOGW(TAG, "!! 紧急全关 — 所有执行器已断电 !!");
    }

    return ret;
}

bool actuator_get(uint8_t channel)
{
    if (channel > 15) {
        return false;
    }
    return (s_actuator_states >> channel) & 0x01;
}

uint16_t actuator_get_bitmap(void)
{
    return s_actuator_states;
}
