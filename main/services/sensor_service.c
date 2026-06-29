/**
 * @file sensor_service.c
 * @brief 传感器采集服务实现
 *
 * 创建 FreeRTOS 任务，按 APP_CFG_MODBUS_POLL_MS (2秒) 间隔轮询 Modbus 传感器。
 *
 * 【当前已实现的传感器】
 *   - 温湿度变送器 (导轨式 V1.4)
 *     从站地址: 0x01, 功能码: 0x03
 *     寄存器 0x0000 = 湿度 (uint16, /10.0 → %RH)
 *     寄存器 0x0001 = 温度 (int16,  /10.0 → ℃, 负数用补码)
 *     一次读取 2 个寄存器: reg_addr=0x0000, count=2
 *
 *   - 氧气传感器 OCS-3F3.0 (经 RP-485 转接板)
 *     从站地址: 0x16, 功能码: 0x04 (读输入寄存器)
 *     寄存器 0x0068 = 氧浓度 (uint16, /10.0 → %)
 *     寄存器 0x0069 = 氧流量 (uint16, /10.0 → L/min)
 *     一次读取 2 个寄存器: reg_addr=0x0068, count=2
 *
 *   - 气压传感器 (赫斯曼 BCP-1R V4.1)
 *     从站地址: 0x03, 功能码: 0x03, 无校验位, 波特率 9600
 *     采用整数法读取 (寄存器 0x0002~0x0004):
 *       0x0002 = 单位编号 (uint16): 1=KPa
 *       0x0003 = 小数位数 (uint16): 0=无, 1=1位, 2=2位, 3=3位, 4=4位
 *       0x0004 = 测量值 (int16, 有符号): 原始整数
 *     换算: pressure = raw_int / pow(10, decimal_places)
 *     一次读取 3 个寄存器: reg_addr=0x0002, count=3
 */
#include "sensor_service.h"
#include "common/app_config.h"
#include "common/app_events.h"
#include "services/bus_scheduler.h"
#include "services/cabin_fsm.h"
#include "drivers/modbus_master.h"
#include "drivers/screen_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <math.h>
#include <stdint.h>

static const char *TAG = "sensor";

/* ==================== 温湿度传感器 Modbus 参数 ==================== */

#define TH_SENSOR_ADDR      0x01    /* 温湿度传感器从站地址（默认） */
#define TH_REG_START         0x0000  /* 起始寄存器：湿度 */
#define TH_REG_COUNT         2       /* 读 2 个寄存器：[0]=湿度, [1]=温度 */

/* ==================== 氧气传感器 Modbus 参数 (RP-485 转接板) ==================== */

#define O2_SENSOR_ADDR      0x16    /* 485转接板默认从站地址 */
#define O2_REG_START        0x0068  /* 起始寄存器：氧浓度 */
#define O2_REG_COUNT        2       /* 读 2 个寄存器：[0]=浓度, [1]=流量 */

/* ==================== 气压传感器 Modbus 参数 (赫斯曼 BCP-1R V4.1) ==================== */

/*
 * 从站地址: 0x03, 功能码: 0x03, 无校验位, 波特率 9600
 *
 * 采用整数法，一次读取 3 个寄存器:
 *   寄存器 0x0002 = 单位编号 (uint16): 1=KPa, 0=MPa, 2=Pa, 3=Bar ...
 *   寄存器 0x0003 = 小数位数 (uint16): 0=无, 1=0.0, 2=0.00, 3=0.000, 4=0.0000
 *   寄存器 0x0004 = 测量值 (int16): 有符号 16 位整数
 *
 * 换算公式: pressure = measurement_int / 10^decimal_places
 *   例: 单位=KPa(1), 小数位=2, 测量值=1001 → 10.01 KPa
 *
 * 注: 浮点法(0x0016 寄存器, IEEE754 大端)也可使用，但整数法更直观可靠，
 *     无字节序歧义，且一次读完所有换算参数，无需额外查询。
 */
#define PRESSURE_SENSOR_ADDR    0x03    /* 气压传感器从站地址 */
#define PRESSURE_REG_START      0x0002  /* 单位寄存器起始地址 */
#define PRESSURE_REG_COUNT      3       /* 读 3 个寄存器: [0]=单位, [1]=小数位, [2]=测量值 */

/* ==================== 传感器数据缓存 ==================== */

static sensor_data_t s_sensor_data = {0};
static SemaphoreHandle_t s_data_mutex = NULL;

/* ==================== 采集任务 ==================== */

/**
 * @brief 传感器轮询任务
 *
 * 每 APP_CFG_MODBUS_POLL_MS (2秒) 执行一次：
 *   1. 通过 bus_scheduler 获取 RS485 总线
 *   2. 读取温湿度传感器 (地址 0x01, 寄存器 0x0000~0x0001)
 *   3. 读取气压传感器 (地址 0x03, 寄存器 0x0002~0x0004, 整数法)
 *   4. 读取氧浓度传感器 (地址 0x16, 寄存器 0x0068~0x0069)
 *   5. 释放总线，通知其他服务数据已就绪
 */
static void sensor_task(void *arg)
{
    uint16_t raw[4];            /* Modbus 寄存器原始值缓冲 */
    uint32_t fail_count = 0;    /* 连续失败计数，用于离线检测 */

    ESP_LOGI(TAG, "传感器采集任务启动 (轮询间隔 %d ms)", APP_CFG_MODBUS_POLL_MS);

    while (1) {
        /* ---- 获取 RS485 总线 ---- */
        if (bus_acquire(BUS_USER_SENSOR, 2000) != ESP_OK) {
            ESP_LOGW(TAG, "获取 485 总线超时，跳过本轮采集");
            vTaskDelay(pdMS_TO_TICKS(APP_CFG_MODBUS_POLL_MS));
            continue;
        }

        /* ---- 1. 读取温湿度传感器 ---- */
        /*
         * 根据温湿度变送器手册 V1.4:
         *   功能码 0x03, 从站地址 0x01
         *   寄存器 0x0000 = 湿度 (unsigned, raw/10.0 → %RH)
         *   寄存器 0x0001 = 温度 (signed,   raw/10.0 → ℃)
         *
         * 请求帧: 01 03 00 00 00 02 C4 0B
         * 响应帧: 01 03 04 [湿度Hi Lo] [温度Hi Lo] [CRC Lo Hi]
         */
        esp_err_t ret = modbus_master_read_holding(TH_SENSOR_ADDR,
                                                    TH_REG_START,
                                                    TH_REG_COUNT,
                                                    raw);
        if (ret == ESP_OK) {
            /* 加锁更新数据 */
            if (s_data_mutex) xSemaphoreTake(s_data_mutex, portMAX_DELAY);

            /* 湿度: 无符号整数 / 10.0 */
            s_sensor_data.humidity = (float)raw[0] / 10.0f;

            /* 温度: 有符号整数 / 10.0 (传感器用补码表示负数) */
            s_sensor_data.temperature = (float)(int16_t)raw[1] / 10.0f;

            s_sensor_data.valid = true;

            if (s_data_mutex) xSemaphoreGive(s_data_mutex);

            fail_count = 0;

            ESP_LOGI(TAG, "温湿度: %.1f℃, %.1f%%RH (raw: 0x%04X, 0x%04X)",
                     s_sensor_data.temperature, s_sensor_data.humidity,
                     raw[1], raw[0]);
        } else {
            fail_count++;
            ESP_LOGW(TAG, "温湿度传感器读取失败: %s (连续 %"PRIu32" 次)",
                     esp_err_to_name(ret), fail_count);
        }

        /* 帧间等待: 让总线彻底静默, 防止上一帧残留数据污染下一次接收 */
        vTaskDelay(pdMS_TO_TICKS(50));

        /* ---- 2. 读取气压传感器 (赫斯曼 BCP-1R, 整数法) ----
         * 从站地址 0x03, 功能码 0x03, 波特率 9600, 无校验位
         * 一次读 3 个寄存器: 0x0002=单位, 0x0003=小数位, 0x0004=测量值
         *
         * 请求帧 (CRC 由驱动自算): 03 03 00 02 00 03 64 08
         * 响应帧: 03 03 06 [单位Hi Lo] [小数位Hi Lo] [测量值Hi Lo] [CRC Lo Hi]
         *
         * 换算: pressure = measurement_int / 10^decimal_places
         *       本机配置: 单位=KPa(1), 小数位=2(0.00精度)
         * 例: 测量值=1001, 小数位=2 → 10.01 KPa
         */
        {
            uint16_t press_raw[PRESSURE_REG_COUNT]; /* [0]=单位, [1]=小数位, [2]=测量值 */
            ret = modbus_master_read_holding(PRESSURE_SENSOR_ADDR,
                                             PRESSURE_REG_START,
                                             PRESSURE_REG_COUNT,
                                             press_raw);
            if (ret == ESP_OK) {
                uint16_t unit_code  = press_raw[0];   /* 单位编号: 1=KPa */
                uint16_t decimals   = press_raw[1];   /* 小数位数: 0~4 */
                int16_t  raw_int    = (int16_t)press_raw[2]; /* 有符号测量值 */

                /* 单位换算: 只支持 KPa (=1), MPa (=0), Pa (=2) 等
                 * 若单位不是 KPa, 按比例换算 (MPa×1000=KPa, Pa÷1000=KPa) */
                float pressure_kpa;
                switch (unit_code) {
                    case 1:  pressure_kpa = (float)raw_int;     break; /* KPa */
                    case 0:  pressure_kpa = (float)raw_int * 1000.0f; break; /* MPa → KPa */
                    case 2:  pressure_kpa = (float)raw_int / 1000.0f; break; /* Pa  → KPa */
                    default: pressure_kpa = (float)raw_int;     break; /* 默认当 KPa */
                }

                /* 除以 10^decimals 得到最终值 */
                float divisor = 1.0f;
                for (uint8_t i = 0; i < decimals && i < 4; i++) {
                    divisor *= 10.0f;
                }
                pressure_kpa /= divisor;

                /* 合理性检查: 氧舱气压典型范围 0~200 KPa */
                if (pressure_kpa < -10.0f || pressure_kpa > 200.0f) {
                    ESP_LOGW(TAG, "气压数据超范围: %.3f KPa (单位=%u, 小数=%u, raw=%d), 丢弃",
                             pressure_kpa, unit_code, decimals, raw_int);
                } else {
                    if (s_data_mutex) xSemaphoreTake(s_data_mutex, portMAX_DELAY);
                    s_sensor_data.pressure_kpa = pressure_kpa;
                    if (s_data_mutex) xSemaphoreGive(s_data_mutex);

                    cabin_fsm_send_event(FSM_EVT_PRESSURE_UPDATE);

                    ESP_LOGI(TAG, "气压: %.2f KPa (单位=%u, 小数位=%u, raw=%d)",
                             pressure_kpa, unit_code, decimals, raw_int);
                }
            } else {
                ESP_LOGW(TAG, "气压传感器读取失败: %s", esp_err_to_name(ret));
            }
        }

        /* 帧间等待: 防止气压帧残留污染氧气传感器接收 */
        vTaskDelay(pdMS_TO_TICKS(50));

        /* ---- 3. 读取氧浓度传感器 (OCS-3F3.0 经 RP-485 转接板) ----
         * 转接板使用功能码 0x04（读输入寄存器）
         * 寄存器 0x0068 = 氧浓度 (uint16, /10.0 → %)
         * 寄存器 0x0069 = 氧流量 (uint16, /10.0 → L/min)
         * 示例请求帧: 16 04 00 68 00 02 F3 30
         */
        ret = modbus_master_read_input(O2_SENSOR_ADDR,
                                       O2_REG_START,
                                       O2_REG_COUNT,
                                       raw);
        if (ret == ESP_OK) {
            if (s_data_mutex) xSemaphoreTake(s_data_mutex, portMAX_DELAY);

            s_sensor_data.oxygen_percent = (float)raw[0] / 10.0f;
            s_sensor_data.oxygen_flow    = (float)raw[1] / 10.0f;

            if (s_data_mutex) xSemaphoreGive(s_data_mutex);

            ESP_LOGI(TAG, "氧气: 浓度 %.1f%%, 流量 %.1f L/min (raw: 0x%04X, 0x%04X)",
                     s_sensor_data.oxygen_percent, s_sensor_data.oxygen_flow,
                     raw[0], raw[1]);
        } else {
            ESP_LOGW(TAG, "氧气传感器读取失败: %s", esp_err_to_name(ret));
        }

        /* ---- 释放 RS485 总线 ---- */
        bus_release(BUS_USER_SENSOR);

        /* ---- 通知其他服务：传感器数据已就绪 ---- */
        if (g_system_events) {
            xEventGroupSetBits(g_system_events, EVT_SENSOR_DATA_READY);
        }

        /* ---- 等待下一轮采集 ---- */
        vTaskDelay(pdMS_TO_TICKS(APP_CFG_MODBUS_POLL_MS));
    }
}

/* ==================== 公共 API ==================== */

esp_err_t sensor_service_start(void)
{
    /* 创建数据互斥锁 */
    s_data_mutex = xSemaphoreCreateMutex();
    if (s_data_mutex == NULL) {
        ESP_LOGE(TAG, "创建数据互斥锁失败");
        return ESP_FAIL;
    }

    /* 把 screen_feed_rx 注册为 Modbus "非 Modbus 字节" 回调,
     * 让屏幕帧不会被 Modbus 吞掉 (参见 modbus_master.h) */
    modbus_master_register_rx_fallback(screen_feed_rx);

    /* 创建采集任务 (优先级 5, 栈 4096 字节) */
    BaseType_t ret = xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建传感器任务失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "传感器采集服务已启动");
    return ESP_OK;
}

esp_err_t sensor_service_get_data(sensor_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 加锁读取，保证线程安全 */
    if (s_data_mutex) xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    *data = s_sensor_data;
    if (s_data_mutex) xSemaphoreGive(s_data_mutex);

    return s_sensor_data.valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}
