/**
 * @file modbus_master.c
 * @brief Modbus RTU 主站驱动实现（手写帧收发方式）
 *
 * 实现标准 Modbus RTU 协议的帧组装、CRC16 校验、UART 收发。
 * 不依赖 ESP-IDF freemodbus 组件，代码自包含，便于理解和答辩讲解。
 *
 * 【CRC16 算法】
 *   使用预计算查找表（256 × 2 字节 = 512B），比逐位计算快 8 倍。
 *   多项式: 0xA001 (Modbus 标准，即 x^16 + x^15 + x^2 + 1 的反转)
 *
 * 【帧格式 (RTU)】
 *   请求: [地址 1B][功能码 1B][数据 NB][CRC16 2B]
 *   帧间间隔 >= 3.5 字符时间 (9600bps 下约 4ms)
 *
 * 引脚定义参见 common/app_config.h。
 */
#include "modbus_master.h"
#include "common/app_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "modbus";

/* ==================== 非 Modbus 数据回调 ==================== */

static modbus_rx_fallback_cb_t s_rx_fallback_cb = NULL;

/* ==================== Modbus 配置常量 ==================== */

#define MB_RESPONSE_TIMEOUT_MS  1000    /* 等待从站响应超时 (ms) */
#define MB_FRAME_GAP_MS         20      /* 帧间间隔 (ms)，RS485自动模式需足够长以切换方向 */
#define MB_MAX_FRAME_SIZE       256     /* 最大帧长度 (字节) */

/* ==================== CRC16 查找表 ==================== */

/**
 * Modbus CRC16 查找表 (多项式 0xA001)
 * 由标准算法预计算生成，共 256 项
 */
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
};

/* ==================== 内部函数 ==================== */

/**
 * @brief 计算 Modbus CRC16 (查表法)
 *
 * 算法说明：
 *   初始值 0xFFFF，对每个字节：
 *   crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF]
 *   结果为小端序（低字节在前），与 Modbus RTU 帧格式一致
 *
 * @param buf  数据缓冲区
 * @param len  数据长度
 * @return CRC16 值（低字节在 [0]，高字节在 [1]）
 */
static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc16_table[(crc ^ buf[i]) & 0xFF];
    }
    return crc;
}

/**
 * @brief 通过 UART 发送 Modbus 请求帧并接收响应
 *
 * 流程：
 *   1. 清空 UART 接收缓冲（丢弃残留数据）
 *   2. 发送请求帧
 *   3. 等待发送完成
 *   4. 等待帧间间隔后开始接收
 *   5. 接收响应数据（超时退出）
 *   6. 校验响应帧 CRC16
 *
 * @param tx_buf     发送帧（含 CRC）
 * @param tx_len     发送帧长度
 * @param rx_buf     接收缓冲区
 * @param rx_max     接收缓冲区最大长度
 * @param timeout_ms 总超时时间 (ms)
 * @param out_len    [输出] 接收到的有效字节数（仅在返回 ESP_OK 时有效）
 * @return ESP_OK: 成功; ESP_ERR_TIMEOUT / ESP_ERR_INVALID_CRC / ESP_ERR_INVALID_SIZE 等
 */
static esp_err_t modbus_send_and_receive(const uint8_t *tx_buf, int tx_len,
                                         uint8_t *rx_buf, int rx_max,
                                         uint32_t timeout_ms, int *out_len)
{
    /* 1. 把 UART RX 残留字节先交给 fallback 回调 (可能是屏幕触控帧/查询响应),
     *    避免 uart_flush_input 直接丢弃。读不完的部分最终仍然 flush 掉。*/
    {
        size_t buffered = 0;
        uart_get_buffered_data_len(APP_CFG_RS485_UART_NUM, &buffered);
        if (buffered > 0) {
            uint8_t stale[MB_MAX_FRAME_SIZE];
            if (buffered > sizeof(stale)) buffered = sizeof(stale);
            int n = uart_read_bytes(APP_CFG_RS485_UART_NUM, stale,
                                    buffered, 0);
            if (n > 0) {
                ESP_LOGD(TAG, "发送前 UART RX 残留 %d 字节, 转发给 fallback", n);
                if (s_rx_fallback_cb) {
                    s_rx_fallback_cb(stale, (uint16_t)n);
                }
            }
        }
        /* 保底: 再等一段让总线彻底静默, 然后 flush 确保干净状态 */
        vTaskDelay(pdMS_TO_TICKS(10));
        uart_flush_input(APP_CFG_RS485_UART_NUM);
    }

    /* 2. 发送请求帧 */
    int sent = uart_write_bytes(APP_CFG_RS485_UART_NUM, tx_buf, tx_len);
    if (sent != tx_len) {
        ESP_LOGE(TAG, "发送失败: 期望 %d 字节, 实际 %d", tx_len, sent);
        return ESP_ERR_INVALID_SIZE;
    }

    /* 3. 等待发送完成（所有字节从 TX FIFO 移出） */
    esp_err_t ret = uart_wait_tx_done(APP_CFG_RS485_UART_NUM,
                                       pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "等待发送完成超时");
        return ESP_ERR_TIMEOUT;
    }

    /* 4. 等待帧间间隔（3.5 字符时间），让从站开始准备响应 */
    vTaskDelay(pdMS_TO_TICKS(MB_FRAME_GAP_MS));

    /* 5. 接收响应数据 — 分段读取，直到超时无新数据 */
    int total_rx = 0;
    int64_t start_us = esp_timer_get_time();
    int64_t timeout_us = (int64_t)timeout_ms * 1000;

    while (total_rx < rx_max) {
        int rx_len = uart_read_bytes(APP_CFG_RS485_UART_NUM,
                                     rx_buf + total_rx,
                                     rx_max - total_rx,
                                     pdMS_TO_TICKS(50));
        if (rx_len > 0) {
            /* 收到数据，打印原始字节 */
            ESP_LOGD(TAG, "收到 %d 字节: %02X %02X %02X %02X ...",
                     rx_len,
                     rx_buf[total_rx],
                     rx_buf[total_rx + 1],
                     rx_buf[total_rx + 2],
                     rx_buf[total_rx + 3]);
            total_rx += rx_len;
            /* 如果已收到足够数据，再等一小段看有没有后续字节 */
            continue;
        }
        /* 没有新数据了 */
        if (total_rx > 0) {
            break;  /* 已收到部分数据且无后续，认为帧接收完毕 */
        }
        /* 一个字节都没收到，检查是否超时 */
        if ((esp_timer_get_time() - start_us) > timeout_us) {
            /* 超时时把 UART RX 缓冲区里残留的字节也读出来看 */
            size_t stale = 0;
            uart_get_buffered_data_len(APP_CFG_RS485_UART_NUM, &stale);
            if (stale > 0) {
                uint8_t trash[64];
                int n = uart_read_bytes(APP_CFG_RS485_UART_NUM, trash, stale > 64 ? 64 : stale, 0);
                ESP_LOGW(TAG, "超时后 UART 残留 %d 字节: %02X %02X %02X ...",
                         n, trash[0], trash[1], trash[2]);
            }
            ESP_LOGW(TAG, "响应超时 (%"PRIu32"ms 内未收到任何数据)", timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
    }

    /* 最少需要 5 字节：地址(1) + 功能码(1) + 数据(至少1) + CRC(2) */
    if (total_rx < 5) {
        ESP_LOGW(TAG, "响应帧过短: %d 字节", total_rx);
        return ESP_ERR_INVALID_SIZE;
    }

    /* 6. CRC16 校验 */
    uint16_t calc_crc = modbus_crc16(rx_buf, total_rx - 2);
    uint16_t recv_crc = (uint16_t)(rx_buf[total_rx - 1] << 8) | rx_buf[total_rx - 2];
    if (calc_crc != recv_crc) {
        ESP_LOGW(TAG, "CRC 校验失败: 计算=0x%04X, 接收=0x%04X", calc_crc, recv_crc);
        /* 收到的很可能是屏幕触控帧/查询响应(与本次 Modbus 响应交叠), 转发给 fallback */
        if (s_rx_fallback_cb) {
            ESP_LOGD(TAG, "转发 %d 字节给 fallback 解析", total_rx);
            s_rx_fallback_cb(rx_buf, (uint16_t)total_rx);
        }
        return ESP_ERR_INVALID_CRC;
    }

    /* 检查 Modbus 异常响应 (功能码最高位置1) */
    if (rx_buf[1] & 0x80) {
        ESP_LOGW(TAG, "从站返回异常: 功能码=0x%02X, 异常码=0x%02X",
                 rx_buf[1], rx_buf[2]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    *out_len = total_rx;
    return ESP_OK;
}

/* ==================== 公共 API ==================== */

esp_err_t modbus_master_init(void)
{
    /* UART 硬件已由 bsp_uart1_rs485_init() 初始化 */
    /* 清空一次接收缓冲，确保干净状态 */
    uart_flush_input(APP_CFG_RS485_UART_NUM);

    ESP_LOGI(TAG, "Modbus RTU 主站初始化完成 (手写帧模式)");
    return ESP_OK;
}

esp_err_t modbus_master_register_rx_fallback(modbus_rx_fallback_cb_t cb)
{
    s_rx_fallback_cb = cb;
    return ESP_OK;
}

esp_err_t modbus_master_read_holding(uint8_t slave_addr, uint16_t reg_addr,
                                     uint16_t reg_count, uint16_t *data)
{
    if (data == NULL || reg_count == 0 || reg_count > 125) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * 组装功能码 0x03 请求帧 (8 字节):
     *   [从站地址] [0x03] [寄存器起始地址Hi] [Lo] [寄存器数量Hi] [Lo] [CRC_Lo] [CRC_Hi]
     */
    uint8_t tx_buf[8];
    tx_buf[0] = slave_addr;
    tx_buf[1] = 0x03;
    tx_buf[2] = (reg_addr >> 8) & 0xFF;
    tx_buf[3] = reg_addr & 0xFF;
    tx_buf[4] = (reg_count >> 8) & 0xFF;
    tx_buf[5] = reg_count & 0xFF;

    uint16_t crc = modbus_crc16(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;         /* CRC 低字节在前 */
    tx_buf[7] = (crc >> 8) & 0xFF;

    /* 打印发送帧用于调试 */
    ESP_LOGI(TAG, "TX [0x03→0x%02X]: %02X %02X %02X %02X %02X %02X %02X %02X",
             slave_addr,
             tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
             tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7]);

    /*
     * 预期响应帧:
     *   [从站地址] [0x03] [字节数=reg_count*2] [数据...] [CRC_Lo] [CRC_Hi]
     *   总长度 = 3 + reg_count*2 + 2 = 5 + reg_count*2
     */
    uint8_t rx_buf[MB_MAX_FRAME_SIZE];
    int expected_len = 5 + reg_count * 2;
    int rx_len = 0;

    esp_err_t err = modbus_send_and_receive(tx_buf, 8, rx_buf, expected_len + 10,
                                             MB_RESPONSE_TIMEOUT_MS, &rx_len);
    if (err != ESP_OK) {
        return err;
    }

    /* 验证响应帧内容 */
    if (rx_buf[0] != slave_addr || rx_buf[1] != 0x03) {
        ESP_LOGW(TAG, "响应帧地址/功能码不匹配: addr=0x%02X func=0x%02X",
                 rx_buf[0], rx_buf[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t byte_count = rx_buf[2];
    if (byte_count != reg_count * 2) {
        ESP_LOGW(TAG, "响应数据长度不匹配: 期望 %d, 实际 %d",
                 reg_count * 2, byte_count);
        return ESP_ERR_INVALID_SIZE;
    }

    /* 提取寄存器数据 — 每个寄存器 2 字节，高字节在前 (big-endian) */
    for (uint16_t i = 0; i < reg_count; i++) {
        data[i] = (uint16_t)(rx_buf[3 + i * 2] << 8) | rx_buf[4 + i * 2];
    }

    /* 打印接收帧用于调试 */
    ESP_LOGI(TAG, "RX [0x03←0x%02X] (%d字节): %02X %02X %02X ... [CRC OK]",
             slave_addr, rx_len,
             rx_buf[0], rx_buf[1], rx_buf[2]);

    ESP_LOGD(TAG, "读保持寄存器成功: addr=0x%02X, reg=0x%04X, count=%d",
             slave_addr, reg_addr, reg_count);
    return ESP_OK;
}

esp_err_t modbus_master_read_input(uint8_t slave_addr, uint16_t reg_addr,
                                    uint16_t reg_count, uint16_t *data)
{
    if (data == NULL || reg_count == 0 || reg_count > 125) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * 功能码 0x04 — 读输入寄存器
     * 帧格式与 0x03 完全一致，只是功能码不同
     */
    uint8_t tx_buf[8];
    tx_buf[0] = slave_addr;
    tx_buf[1] = 0x04;
    tx_buf[2] = (reg_addr >> 8) & 0xFF;
    tx_buf[3] = reg_addr & 0xFF;
    tx_buf[4] = (reg_count >> 8) & 0xFF;
    tx_buf[5] = reg_count & 0xFF;

    uint16_t crc = modbus_crc16(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;

    /* 调试: 打印发送帧 */
    ESP_LOGI(TAG, "TX [0x04→0x%02X]: %02X %02X %02X %02X %02X %02X %02X %02X",
             slave_addr,
             tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
             tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7]);

    uint8_t rx_buf[MB_MAX_FRAME_SIZE];
    int expected_len = 5 + reg_count * 2;
    int rx_len = 0;

    esp_err_t err = modbus_send_and_receive(tx_buf, 8, rx_buf, expected_len + 10,
                                             MB_RESPONSE_TIMEOUT_MS, &rx_len);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RX [0x04←0x%02X]: 失败 (%s)", slave_addr, esp_err_to_name(err));
        return err;
    }

    /* 调试: 打印接收帧 */
    ESP_LOGD(TAG, "RX [0x04←0x%02X] (%d字节):", slave_addr, rx_len);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, rx_buf, rx_len, ESP_LOG_DEBUG);

    if (rx_buf[0] != slave_addr || rx_buf[1] != 0x04) {
        ESP_LOGW(TAG, "响应帧地址/功能码不匹配: addr=0x%02X func=0x%02X",
                 rx_buf[0], rx_buf[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t byte_count = rx_buf[2];
    if (byte_count != reg_count * 2) {
        ESP_LOGW(TAG, "响应数据长度不匹配: 期望 %d, 实际 %d",
                 reg_count * 2, byte_count);
        return ESP_ERR_INVALID_SIZE;
    }

    for (uint16_t i = 0; i < reg_count; i++) {
        data[i] = (uint16_t)(rx_buf[3 + i * 2] << 8) | rx_buf[4 + i * 2];
    }

    return ESP_OK;
}

esp_err_t modbus_master_write_single(uint8_t slave_addr, uint16_t reg_addr,
                                     uint16_t value)
{
    /*
     * 功能码 0x06 — 写单个保持寄存器
     * 请求帧 (8 字节): [地址] [0x06] [寄存器地址Hi] [Lo] [写入值Hi] [Lo] [CRC_Lo] [CRC_Hi]
     * 响应帧: 与请求帧相同（原样回显）
     */
    uint8_t tx_buf[8];
    tx_buf[0] = slave_addr;
    tx_buf[1] = 0x06;
    tx_buf[2] = (reg_addr >> 8) & 0xFF;
    tx_buf[3] = reg_addr & 0xFF;
    tx_buf[4] = (value >> 8) & 0xFF;
    tx_buf[5] = value & 0xFF;

    uint16_t crc = modbus_crc16(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;

    uint8_t rx_buf[MB_MAX_FRAME_SIZE];
    int rx_len = 0;

    esp_err_t err = modbus_send_and_receive(tx_buf, 8, rx_buf, 8 + 4,
                                             MB_RESPONSE_TIMEOUT_MS, &rx_len);
    if (err != ESP_OK) {
        return err;
    }

    /* 验证回显：响应帧应与请求帧完全一致（CRC 已在 send_and_receive 中校验） */
    if (memcmp(tx_buf, rx_buf, 6) != 0) {
        ESP_LOGW(TAG, "写寄存器回显不匹配");
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGD(TAG, "写寄存器成功: addr=0x%02X, reg=0x%04X, value=0x%04X",
             slave_addr, reg_addr, value);
    return ESP_OK;
}
