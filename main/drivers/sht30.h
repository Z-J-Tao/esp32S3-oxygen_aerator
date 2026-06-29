/**
 * @file sht30.h
 * @brief SHT30 温湿度传感器驱动 (I2C)
 */
#ifndef SHT30_H
#define SHT30_H

#include "esp_err.h"

/**
 * @brief 初始化 SHT30 (软复位 + 验证通信)
 */
esp_err_t sht30_init(void);

/**
 * @brief 读取温湿度 (单次高精度模式)
 * @param temperature  输出温度 (°C)
 * @param humidity     输出相对湿度 (%RH)
 */
esp_err_t sht30_read(float *temperature, float *humidity);

#endif /* SHT30_H */
