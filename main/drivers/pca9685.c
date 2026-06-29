/**
 * @file pca9685.c
 * @brief PCA9685PW I2C PWM 控制器驱动实现
 *
 * 【I2C 通信】
 *   写寄存器: START → [0x80|W] ACK → [reg] ACK → [data...] ACK → STOP
 *   使用 ESP-IDF 旧版 I2C 驱动 (i2c_master_write_to_device)
 *
 * 【二值控制】
 *   本系统用 PCA9685 驱动继电器（通/断），不用 PWM 调光：
 *   - 全开: LED_ON_H  bit4=1 (full-on)
 *   - 全关: LED_OFF_H bit4=1 (full-off，优先级高于 full-on)
 */
#include "pca9685.h"
#include "common/app_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pca9685";

/* ============================================================
 * PCA9685 寄存器地址
 * ============================================================ */

#define PCA9685_MODE1           0x00
#define PCA9685_MODE2           0x01
#define PCA9685_LED0_ON_L       0x06    /* 每通道 4 字节: ON_L, ON_H, OFF_L, OFF_H */
#define PCA9685_ALL_LED_ON_L    0xFA
#define PCA9685_ALL_LED_ON_H    0xFB
#define PCA9685_ALL_LED_OFF_L   0xFC
#define PCA9685_ALL_LED_OFF_H   0xFD
#define PCA9685_PRE_SCALE       0xFE

/* MODE1 位定义 */
#define MODE1_RESTART           0x80
#define MODE1_AI                0x20    /* 地址自动递增 */
#define MODE1_SLEEP             0x10

/* I2C 超时 */
#define I2C_TIMEOUT_MS          100

/* ============================================================
 * I2C 读写辅助
 * ============================================================ */

static esp_err_t pca9685_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(APP_CFG_I2C_NUM, APP_CFG_PCA9685_ADDR,
                                      buf, sizeof(buf),
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t pca9685_write_burst(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buf[6];  /* reg + 最多 5 字节数据 */
    buf[0] = reg;
    for (uint8_t i = 0; i < len && i < 5; i++) {
        buf[1 + i] = data[i];
    }
    return i2c_master_write_to_device(APP_CFG_I2C_NUM, APP_CFG_PCA9685_ADDR,
                                      buf, 1 + len,
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/* ============================================================
 * 公开 API
 * ============================================================ */

esp_err_t pca9685_init(void)
{
    esp_err_t ret;

    /* 步骤 1: 进入 SLEEP 模式 (PRE_SCALE 只能在 SLEEP 下修改) */
    ret = pca9685_write(PCA9685_MODE1, MODE1_SLEEP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法进入 SLEEP 模式，I2C 通信失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 步骤 2: 设置 PWM 频率 50Hz
     * PRE_SCALE = round(25MHz / (4096 × 50Hz)) - 1 = 121 */
    ret = pca9685_write(PCA9685_PRE_SCALE, 121);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置 PRE_SCALE 失败");
        return ret;
    }

    /* 步骤 3: 退出 SLEEP，使能 RESTART + AI (自动地址递增) */
    ret = pca9685_write(PCA9685_MODE1, MODE1_RESTART | MODE1_AI);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "退出 SLEEP 失败");
        return ret;
    }

    /* RESTART 后等待 500µs 让振荡器稳定 */
    vTaskDelay(pdMS_TO_TICKS(1));

    /* 步骤 4: 关闭所有通道 (ALL_LED_OFF_H bit4=1 → 全部 full-off) */
    ret = pca9685_write(PCA9685_ALL_LED_OFF_H, 0x10);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "关闭所有通道失败");
        return ret;
    }
    /* 清除 full-on */
    pca9685_write(PCA9685_ALL_LED_ON_H, 0x00);

    ESP_LOGI(TAG, "PCA9685 初始化完成 (addr=0x%02X, 50Hz, 全部通道已关闭)",
             APP_CFG_PCA9685_ADDR);
    return ESP_OK;
}

esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = PCA9685_LED0_ON_L + channel * 4;
    uint8_t data[4] = {
        (uint8_t)(on & 0xFF),           /* ON_L */
        (uint8_t)((on >> 8) & 0x0F),    /* ON_H */
        (uint8_t)(off & 0xFF),          /* OFF_L */
        (uint8_t)((off >> 8) & 0x0F),   /* OFF_H */
    };

    return pca9685_write_burst(reg, data, 4);
}

esp_err_t pca9685_channel_on(uint8_t channel)
{
    if (channel > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    /* full-on: ON_H bit4=1, OFF_H bit4=0 */
    uint8_t reg = PCA9685_LED0_ON_L + channel * 4;
    uint8_t data[4] = {
        0x00,   /* ON_L */
        0x10,   /* ON_H  bit4=1 (full-on) */
        0x00,   /* OFF_L */
        0x00,   /* OFF_H bit4=0 (清除 full-off) */
    };

    return pca9685_write_burst(reg, data, 4);
}

esp_err_t pca9685_channel_off(uint8_t channel)
{
    if (channel > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    /* full-off: OFF_H bit4=1 (优先级最高), ON_H bit4=0 */
    uint8_t reg = PCA9685_LED0_ON_L + channel * 4;
    uint8_t data[4] = {
        0x00,   /* ON_L */
        0x00,   /* ON_H  bit4=0 (清除 full-on) */
        0x00,   /* OFF_L */
        0x10,   /* OFF_H bit4=1 (full-off) */
    };

    return pca9685_write_burst(reg, data, 4);
}

esp_err_t pca9685_all_off(void)
{
    /* ALL_LED_OFF_H bit4=1 → 全部 full-off */
    esp_err_t ret = pca9685_write(PCA9685_ALL_LED_OFF_H, 0x10);
    /* 清除 ALL_LED full-on */
    pca9685_write(PCA9685_ALL_LED_ON_H, 0x00);
    return ret;
}
