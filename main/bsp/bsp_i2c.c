/**
 * @file bsp_i2c.c
 * @brief I2C 总线初始化实现
 *
 * I2C0 主机，连接 PCA9685PW (0x40)。
 * 引脚: SDA=GPIO1, SCL=GPIO2, 外部 3.3kΩ 上拉。
 */
#include "bsp_i2c.h"
#include "common/app_config.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "bsp_i2c";

esp_err_t bsp_i2c_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = APP_CFG_I2C_SDA_PIN,     /* GPIO1 */
        .scl_io_num       = APP_CFG_I2C_SCL_PIN,     /* GPIO2 */
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,     /* PCB 已有外部 3.3k 上拉 */
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = APP_CFG_I2C_FREQ_HZ,     /* 100kHz */
    };

    esp_err_t ret = i2c_param_config(APP_CFG_I2C_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 参数配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(APP_CFG_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 驱动安装失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C0 初始化完成 (SDA=%d, SCL=%d, %dkHz)",
             APP_CFG_I2C_SDA_PIN, APP_CFG_I2C_SCL_PIN, APP_CFG_I2C_FREQ_HZ / 1000);
    return ESP_OK;
}
