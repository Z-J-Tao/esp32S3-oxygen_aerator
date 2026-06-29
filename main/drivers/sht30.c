/**
 * @file sht30.c
 * @brief SHT30 温湿度传感器驱动实现
 *
 * 【通信协议】
 *   单次高精度测量:
 *     写命令: START → [0x44<<1|W] ACK → [0x24] ACK → [0x00] ACK → STOP
 *     等待 ~15ms
 *     读数据: START → [0x44<<1|R] ACK → [T_MSB] [T_LSB] [T_CRC] [H_MSB] [H_LSB] [H_CRC] → STOP
 */
#include "sht30.h"
#include "common/app_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sht30";

#define I2C_TIMEOUT_MS      100

/* SHT30 命令 */
#define SHT30_CMD_MEASURE_H  0x2400  /* 单次, 高重复性, 不拉伸时钟 */
#define SHT30_CMD_SOFT_RESET 0x30A2

static uint8_t sht30_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static esp_err_t sht30_send_cmd(uint16_t cmd)
{
    uint8_t buf[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    return i2c_master_write_to_device(APP_CFG_I2C_NUM, APP_CFG_SHT30_ADDR,
                                      buf, sizeof(buf),
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t sht30_init(void)
{
    /* 软复位 */
    esp_err_t ret = sht30_send_cmd(SHT30_CMD_SOFT_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT30 软复位失败: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(2));  /* 复位需要 ~1.5ms */

    /* 试读一次, 验证通信 */
    float t, h;
    ret = sht30_read(&t, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT30 首次读取失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SHT30 初始化完成 (addr=0x%02X, T=%.1f°C, RH=%.1f%%)",
             APP_CFG_SHT30_ADDR, t, h);
    return ESP_OK;
}

esp_err_t sht30_read(float *temperature, float *humidity)
{
    if (temperature == NULL || humidity == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 发送单次高精度测量命令 */
    esp_err_t ret = sht30_send_cmd(SHT30_CMD_MEASURE_H);
    if (ret != ESP_OK) return ret;

    /* 等待测量完成 (高精度 ~15ms) */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 读取 6 字节: T_MSB, T_LSB, T_CRC, H_MSB, H_LSB, H_CRC */
    uint8_t data[6] = {0};
    ret = i2c_master_read_from_device(APP_CFG_I2C_NUM, APP_CFG_SHT30_ADDR,
                                      data, sizeof(data),
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT30 读取失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* CRC 校验 */
    if (sht30_crc8(data, 2) != data[2] || sht30_crc8(data + 3, 2) != data[5]) {
        ESP_LOGE(TAG, "SHT30 CRC 校验失败");
        return ESP_ERR_INVALID_CRC;
    }

    /* 转换: 温度 T = -45 + 175 × (raw / 65535)
     *       湿度 H = 100 × (raw / 65535) */
    uint16_t t_raw = ((uint16_t)data[0] << 8) | data[1];
    uint16_t h_raw = ((uint16_t)data[3] << 8) | data[4];

    *temperature = -45.0f + 175.0f * ((float)t_raw / 65535.0f);
    *humidity    = 100.0f * ((float)h_raw / 65535.0f);

    return ESP_OK;
}
