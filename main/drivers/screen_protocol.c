/**
 * @file screen_protocol.c
 * @brief 尚视界串口屏 XFD/SFD 协议驱动实现
 *
 * 【帧格式 — 发送 (MCU → 屏幕)】
 *   EE [addr_H] [addr_L] [cmd bytes...] [CRC_H] [CRC_L] FF FC FF FF
 *   CRC16 计算范围: 仅 cmd bytes (不含 EE 和地址)
 *
 * 【帧格式 — 触控上报 (屏幕 → MCU)】
 *   CC [addr_H addr_L] [cmd=0x06] [page_H page_L] [widget_H widget_L]
 *      [X_H X_L] [Y_H Y_L] [event_H event_L] [CRC_H CRC_L] FF FC FF FF
 *   CRC16 计算范围: cmd + page + widget + X + Y + event (不含 CC 和地址)
 *   典型长度: CC(1) + 地址(2) + cmd(1) + data(10) + CRC(2) + 帧尾(4) = 20 字节
 *
 * 【注意事项】
 *   - 调用发送函数前必须已通过 bus_scheduler 获取 485 总线
 *   - 波特率 9600, 地址 0x0002, CRC16 已启用
 */
#include "screen_protocol.h"
#include "common/app_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "screen_proto";

/* ============================================================
 * 触控回调 / 页面查询回调
 * ============================================================ */

static screen_touch_cb_t       s_touch_cb = NULL;
static screen_page_update_cb_t s_page_cb  = NULL;

/* ============================================================
 * 接收解析状态机
 *
 * 支持两种上行帧:
 *   - 0xCC 开头: 触控上报帧
 *   - 0xEE 开头: 指令响应帧 (当前只处理 cmd=0x06 查询当前页响应)
 * 两种帧尾均为 FF FC FF FF
 * ============================================================ */

/* 状态定义 */
typedef enum {
    RX_WAIT_HEADER,         /* 等待帧头 (0xCC 或 0xEE) */
    RX_RECV_DATA,           /* 接收数据，等待帧尾 */
} rx_state_t;

/* 接收缓冲区 */
static uint8_t  s_rx_buf[SCREEN_MAX_FRAME_SIZE];
static uint16_t s_rx_len = 0;
static rx_state_t s_rx_state = RX_WAIT_HEADER;
static uint8_t  s_rx_header = 0;  /* 记录当前帧的帧头: 0xCC 或 0xEE */

/* 帧尾匹配计数 */
static uint8_t s_tail_match = 0;
static const uint8_t TAIL_BYTES[4] = {0xFF, 0xFC, 0xFF, 0xFF};

/* ============================================================
 * CRC-16/MODBUS 计算
 * 多项式: x^16 + x^15 + x^2 + 1 (反射形式 0xA001)
 * 初始值: 0xFFFF
 * ============================================================ */

static uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ============================================================
 * 发送帧构建与发送
 *
 * 帧结构: EE [addr_H addr_L] [payload...] [CRC_H CRC_L] [FF FC FF FF]
 * CRC 计算范围: 仅 payload (不含帧头 EE 和地址字节)
 * ============================================================ */

static esp_err_t screen_send_frame(const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len == 0 || payload_len > SCREEN_MAX_FRAME_SIZE - 10) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t frame[SCREEN_MAX_FRAME_SIZE];
    uint16_t idx = 0;

    /* 帧头 */
    frame[idx++] = SCREEN_FRAME_HEADER;  /* 0xEE */

    /* 地址 (2字节) */
    frame[idx++] = SCREEN_ADDR_H;        /* 0x00 */
    frame[idx++] = SCREEN_ADDR_L;        /* 0x02 */

    /* 指令+数据 */
    memcpy(&frame[idx], payload, payload_len);
    idx += payload_len;

    /* CRC16 — 计算范围: 仅 payload 部分 (不含帧头 EE 和地址) */
    uint16_t crc = crc16_modbus(&frame[3], payload_len);
    frame[idx++] = (uint8_t)(crc >> 8);   /* CRC 高字节 */
    frame[idx++] = (uint8_t)(crc & 0xFF); /* CRC 低字节 */

    /* 帧尾 */
    frame[idx++] = 0xFF;
    frame[idx++] = 0xFC;
    frame[idx++] = 0xFF;
    frame[idx++] = 0xFF;

    /* 调试: 打印发送帧的完整 hex dump (仅 DEBUG 级别) */
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, frame, idx, ESP_LOG_DEBUG);

    /* 通过 UART1 发送 */
    int sent = uart_write_bytes(APP_CFG_RS485_UART_NUM, frame, idx);
    if (sent < 0) {
        ESP_LOGE(TAG, "UART 发送失败");
        return ESP_FAIL;
    }

    /* 等待发送完成 */
    esp_err_t ret = uart_wait_tx_done(APP_CFG_RS485_UART_NUM, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "UART 发送等待超时");
    }

    return ESP_OK;
}

/* ============================================================
 * 触控帧解析
 *
 * 接收到完整帧后 (s_rx_buf 中, 不含帧头 CC 和帧尾 FF FC FF FF):
 *   [addr_H addr_L] [cmd=0x06] [page_H page_L] [widget_H widget_L]
 *   [X_H X_L] [Y_H Y_L] [event_H event_L] [CRC_H CRC_L]
 *   共 15 字节 (地址2 + cmd1 + 数据10 + CRC2)
 *
 * CRC16 覆盖范围: cmd 字节起到 event 末 (偏移 2 到 len-3)
 * ============================================================ */

/* 触控帧固定字段偏移 (相对 s_rx_buf 起始, 不含 CC 帧头) */
#define TOUCH_OFFSET_CMD     2   /* 0x06 */
#define TOUCH_OFFSET_PAGE    3
#define TOUCH_OFFSET_WIDGET  5
#define TOUCH_OFFSET_X       7
#define TOUCH_OFFSET_Y       9
#define TOUCH_OFFSET_EVENT  11
#define TOUCH_CMD_EVENT     0x06
#define TOUCH_MIN_LEN       15   /* addr2 + cmd1 + data10 + crc2 */

static void parse_touch_frame(void)
{
    if (s_rx_len < TOUCH_MIN_LEN) {
        ESP_LOGW(TAG, "触控帧长度不足: %d (期望 >= %d)", s_rx_len, TOUCH_MIN_LEN);
        return;
    }

    /* 只处理 cmd=0x06 的触控事件帧，其他上报帧静默忽略 */
    if (s_rx_buf[TOUCH_OFFSET_CMD] != TOUCH_CMD_EVENT) {
        ESP_LOGD(TAG, "忽略非触控帧: cmd=0x%02X", s_rx_buf[TOUCH_OFFSET_CMD]);
        return;
    }

    /* 验证 CRC16 — 计算范围: cmd 到 event 末 (偏移 2 到 s_rx_len-3) */
    uint16_t crc_data_len = s_rx_len - 2 - TOUCH_OFFSET_CMD;  /* 去掉地址2+CRC2 */
    uint16_t crc_calc = crc16_modbus(&s_rx_buf[TOUCH_OFFSET_CMD], crc_data_len);
    uint16_t crc_recv = ((uint16_t)s_rx_buf[s_rx_len - 2] << 8) | s_rx_buf[s_rx_len - 1];

    if (crc_calc != crc_recv) {
        ESP_LOGW(TAG, "触控帧 CRC 校验失败: calc=0x%04X recv=0x%04X", crc_calc, crc_recv);
        return;
    }

    /* 解析触控数据 (跳过开头的 addr + cmd) */
    screen_touch_data_t touch;
    touch.page_id   = ((uint16_t)s_rx_buf[TOUCH_OFFSET_PAGE]   << 8) | s_rx_buf[TOUCH_OFFSET_PAGE + 1];
    touch.widget_id = ((uint16_t)s_rx_buf[TOUCH_OFFSET_WIDGET] << 8) | s_rx_buf[TOUCH_OFFSET_WIDGET + 1];
    touch.x         = ((uint16_t)s_rx_buf[TOUCH_OFFSET_X]      << 8) | s_rx_buf[TOUCH_OFFSET_X + 1];
    touch.y         = ((uint16_t)s_rx_buf[TOUCH_OFFSET_Y]      << 8) | s_rx_buf[TOUCH_OFFSET_Y + 1];
    touch.event     = ((uint16_t)s_rx_buf[TOUCH_OFFSET_EVENT]  << 8) | s_rx_buf[TOUCH_OFFSET_EVENT + 1];

    ESP_LOGD(TAG, "触控: page=%d widget=0x%04X x=%d y=%d event=%d",
             touch.page_id, touch.widget_id, touch.x, touch.y, touch.event);

    /* 调用回调 */
    if (s_touch_cb) {
        s_touch_cb(&touch);
    }
}

/* ============================================================
 * EE 响应帧解析
 *
 * 接收到完整帧后 (s_rx_buf 中, 不含帧头 EE 和帧尾 FF FC FF FF):
 *   [addr_H addr_L] [cmd] [data...] [CRC_H CRC_L]
 *
 * 当前只处理 cmd=0x06 (查询当前页响应):
 *   [addr_H addr_L] [0x06] [page_H page_L] [CRC_H CRC_L]  共 7 字节
 *
 * CRC16 覆盖范围: cmd 起到 data 末 (偏移 2 到 len-3)
 * ============================================================ */

#define EE_OFFSET_CMD   2    /* addr 占 2 字节, cmd 在偏移 2 */
#define EE_MIN_LEN      5    /* addr2 + cmd1 + crc2 = 最小 5 字节 */

static void parse_ee_frame(void)
{
    if (s_rx_len < EE_MIN_LEN) {
        ESP_LOGW(TAG, "EE 帧长度不足: %d (期望 >= %d)", s_rx_len, EE_MIN_LEN);
        return;
    }

    /* CRC 覆盖 cmd 起到 data 末 */
    uint16_t crc_data_len = s_rx_len - 2 - EE_OFFSET_CMD;  /* 去掉 addr(2) + crc(2) */
    uint16_t crc_calc = crc16_modbus(&s_rx_buf[EE_OFFSET_CMD], crc_data_len);
    uint16_t crc_recv = ((uint16_t)s_rx_buf[s_rx_len - 2] << 8) | s_rx_buf[s_rx_len - 1];

    if (crc_calc != crc_recv) {
        ESP_LOGW(TAG, "EE 帧 CRC 校验失败: calc=0x%04X recv=0x%04X", crc_calc, crc_recv);
        return;
    }

    uint8_t cmd = s_rx_buf[EE_OFFSET_CMD];

    switch (cmd) {
    case 0x06: {
        /* 查询当前页响应: payload = [cmd=0x06] [page_H page_L] */
        if (s_rx_len != 7) {
            ESP_LOGW(TAG, "查询页响应长度异常: %d (期望 7)", s_rx_len);
            return;
        }
        uint16_t page_id = ((uint16_t)s_rx_buf[3] << 8) | s_rx_buf[4];
        ESP_LOGD(TAG, "查询页响应: current_page=%d", page_id);
        if (s_page_cb) {
            s_page_cb(page_id);
        }
        break;
    }
    default:
        ESP_LOGD(TAG, "忽略 EE 响应帧: cmd=0x%02X", cmd);
        break;
    }
}

/* ============================================================
 * 公开 API 实现
 * ============================================================ */

esp_err_t screen_protocol_init(void)
{
    s_rx_state = RX_WAIT_HEADER;
    s_rx_len = 0;
    s_tail_match = 0;
    s_rx_header = 0;
    s_touch_cb = NULL;
    s_page_cb  = NULL;

    ESP_LOGI(TAG, "串口屏协议初始化完成 (addr=0x%02X%02X, CRC16=ON)",
             SCREEN_ADDR_H, SCREEN_ADDR_L);
    return ESP_OK;
}

esp_err_t screen_switch_page(uint16_t page_id)
{
    /* 指令格式: 05 [page_H] [page_L] [delay_H] [delay_L]
     * delay 设为 0 = 立即切换 */
    uint8_t payload[5];
    payload[0] = 0x05;
    payload[1] = (uint8_t)(page_id >> 8);
    payload[2] = (uint8_t)(page_id & 0xFF);
    payload[3] = 0x00;  /* delay 高字节 */
    payload[4] = 0x00;  /* delay 低字节 (0ms) */

    return screen_send_frame(payload, sizeof(payload));
}

esp_err_t screen_set_text(uint16_t widget_id, const char *text)
{
    if (text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t text_len = (uint16_t)strlen(text);

    /* 指令格式: 81 05 [id_H] [id_L] [text bytes...] 00
     * 末尾必须追加 0x00 作为字符串结束符 */
    uint16_t payload_len = 2 + 2 + text_len + 1;  /* cmd(2) + id(2) + text + null */
    if (payload_len > SCREEN_MAX_FRAME_SIZE - 10) {
        ESP_LOGE(TAG, "文本过长: %d 字节", text_len);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t payload[SCREEN_MAX_FRAME_SIZE];
    uint16_t idx = 0;

    payload[idx++] = 0x81;
    payload[idx++] = 0x05;
    payload[idx++] = (uint8_t)(widget_id >> 8);
    payload[idx++] = (uint8_t)(widget_id & 0xFF);
    memcpy(&payload[idx], text, text_len);
    idx += text_len;
    payload[idx++] = 0x00;  /* 字符串结束符 */

    return screen_send_frame(payload, idx);
}

esp_err_t screen_set_val(uint16_t widget_id, uint16_t value)
{
    /* 指令格式: 81 09 [id_H] [id_L] [val_H] [val_L] */
    uint8_t payload[6];
    payload[0] = 0x81;
    payload[1] = 0x09;
    payload[2] = (uint8_t)(widget_id >> 8);
    payload[3] = (uint8_t)(widget_id & 0xFF);
    payload[4] = (uint8_t)(value >> 8);
    payload[5] = (uint8_t)(value & 0xFF);

    return screen_send_frame(payload, sizeof(payload));
}

esp_err_t screen_set_hidden(uint16_t widget_id, bool hidden)
{
    /* 指令格式: 81 0A [id_H] [id_L] [00] [00] [00] [hidden]
     * hidden: 0x00000001=隐藏, 0x00000000=显示
     * 值为 4 字节 uint32 大端序 */
    uint8_t payload[8];
    payload[0] = 0x81;
    payload[1] = 0x0A;
    payload[2] = (uint8_t)(widget_id >> 8);
    payload[3] = (uint8_t)(widget_id & 0xFF);
    payload[4] = 0x00;
    payload[5] = 0x00;
    payload[6] = 0x00;
    payload[7] = hidden ? 0x01 : 0x00;

    return screen_send_frame(payload, sizeof(payload));
}

esp_err_t screen_set_backlight(uint8_t brightness)
{
    /* 指令格式: 07 [val_H] [val_L]
     * val 范围 0~255 */
    uint8_t payload[3];
    payload[0] = 0x07;
    payload[1] = 0x00;
    payload[2] = brightness;

    return screen_send_frame(payload, sizeof(payload));
}

esp_err_t screen_set_font_color(uint16_t widget_id, uint32_t color)
{
    /* 指令格式: 81 06 [id_H] [id_L] [label_state 0x00 0x00] [A] [R] [G] [B]
     * label_state = 0x0000 表示默认状态
     * color 为 32位 ARGB */
    uint8_t payload[10];
    payload[0] = 0x81;
    payload[1] = 0x06;
    payload[2] = (uint8_t)(widget_id >> 8);
    payload[3] = (uint8_t)(widget_id & 0xFF);
    payload[4] = 0x00;  /* label state 高字节 */
    payload[5] = 0x00;  /* label state 低字节 (默认状态) */
    payload[6] = (uint8_t)((color >> 24) & 0xFF);  /* A */
    payload[7] = (uint8_t)((color >> 16) & 0xFF);  /* R */
    payload[8] = (uint8_t)((color >> 8) & 0xFF);   /* G */
    payload[9] = (uint8_t)(color & 0xFF);           /* B */

    return screen_send_frame(payload, sizeof(payload));
}

esp_err_t screen_register_touch_callback(screen_touch_cb_t cb)
{
    s_touch_cb = cb;
    return ESP_OK;
}

esp_err_t screen_register_page_callback(screen_page_update_cb_t cb)
{
    s_page_cb = cb;
    return ESP_OK;
}

esp_err_t screen_query_page(void)
{
    /* 指令格式: 06  (单字节 cmd, 无数据)
     * 期望响应: EE 00 02 06 [page_H page_L] [CRC] FF FC FF FF */
    uint8_t payload[1] = { 0x06 };
    return screen_send_frame(payload, sizeof(payload));
}

void screen_feed_rx(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        switch (s_rx_state) {
        case RX_WAIT_HEADER:
            if (byte == SCREEN_TOUCH_HEADER || byte == SCREEN_FRAME_HEADER) {
                /* 检测到 CC (触控上报) 或 EE (指令响应) 帧头 */
                s_rx_header  = byte;
                s_rx_len     = 0;
                s_tail_match = 0;
                s_rx_state   = RX_RECV_DATA;
            }
            /* 其他字节忽略 */
            break;

        case RX_RECV_DATA:
            /* 缓冲区溢出保护 */
            if (s_rx_len >= SCREEN_MAX_FRAME_SIZE) {
                ESP_LOGW(TAG, "接收缓冲区溢出，丢弃帧");
                s_rx_state = RX_WAIT_HEADER;
                break;
            }

            /* 存入缓冲区 */
            s_rx_buf[s_rx_len++] = byte;

            /* 检测帧尾 FF FC FF FF */
            if (byte == TAIL_BYTES[s_tail_match]) {
                s_tail_match++;
                if (s_tail_match == 4) {
                    /* 帧尾匹配完成，去掉末尾 4 字节帧尾 */
                    s_rx_len -= 4;
                    if (s_rx_header == SCREEN_TOUCH_HEADER) {
                        parse_touch_frame();
                    } else if (s_rx_header == SCREEN_FRAME_HEADER) {
                        parse_ee_frame();
                    }
                    s_rx_state = RX_WAIT_HEADER;
                }
            } else {
                /* 不匹配时检查当前字节是否是帧尾首字节 */
                if (byte == TAIL_BYTES[0]) {
                    s_tail_match = 1;
                } else {
                    s_tail_match = 0;
                }
            }
            break;
        }
    }
}
