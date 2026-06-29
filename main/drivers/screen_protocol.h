/**
 * @file screen_protocol.h
 * @brief 尚视界串口屏通信协议驱动
 *
 * 【硬件连接】
 *   串口屏通过 RS485 总线（UART1）连接，与 Modbus 传感器共用同一路 485。
 *   通信前必须通过 bus_scheduler 获取总线使用权。
 *
 * 【协议概述 — XFD/SFD 协议】
 *   帧头: 0xEE
 *   帧尾: 0xFF 0xFC 0xFF 0xFF
 *   地址模式: 启用，屏幕地址 = 0x0002
 *   CRC16:   启用 (CRC-16/MODBUS)
 *
 *   发送帧格式 (MCU → 屏幕):
 *     EE [addr_H] [addr_L] [cmd] [data...] [CRC_H] [CRC_L] FF FC FF FF
 *
 *   触控上报帧 (屏幕 → MCU):
 *     CC [data...] [CRC_H] [CRC_L] FF FC FF FF
 *     data 内容: page_id(16) + widget_id(16) + X(16) + Y(16) + event(16)
 *
 *   CRC16 计算范围:
 *     发送帧: 仅 cmd+data 部分 (不含帧头 EE、地址、帧尾)
 *     触控帧: 从 CC 之后到 CRC 之前的所有字节
 */
#ifndef SCREEN_PROTOCOL_H
#define SCREEN_PROTOCOL_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 协议配置
 * ============================================================ */

#define SCREEN_ADDR_H           0x00    /* 屏幕 485 地址高字节 */
#define SCREEN_ADDR_L           0x02    /* 屏幕 485 地址低字节 */

#define SCREEN_FRAME_HEADER     0xEE    /* 发送帧头 */
#define SCREEN_FRAME_TAIL       "\xFF\xFC\xFF\xFF"  /* 帧尾 (4字节) */
#define SCREEN_FRAME_TAIL_LEN   4

#define SCREEN_TOUCH_HEADER     0xCC    /* 触控上报帧头 */

#define SCREEN_MAX_FRAME_SIZE   128     /* 单帧最大字节数 */

/* ============================================================
 * 指令码定义 (MCU → 屏幕)
 * ============================================================ */

#define SCREEN_CMD_SWITCH_PAGE  0x05    /* 切换页面 */
#define SCREEN_CMD_SET_BL       0x07    /* 设置背光亮度 (0~255) */
#define SCREEN_CMD_SET_TEXT     0x05    /* 设置控件文本 (子码 0x81 0x05) */
#define SCREEN_CMD_SET_VAL      0x09    /* 设置控件数值 (子码 0x81 0x09) */
#define SCREEN_CMD_SET_HIDDEN   0x0A    /* 设置控件显隐 (子码 0x81 0x0A) */
#define SCREEN_CMD_SET_ENABLE   0x0F    /* 设置控件使能 (子码 0x81 0x0F) */
#define SCREEN_CMD_SET_COLOR1   0x06    /* 设置前景色1 (子码 0x81 0x06) */

/* ============================================================
 * 触控事件类型
 *
 * 实测结论: event 字段是 **每个控件独立的"按下次数"计数器**，
 *          屏幕上此控件每按一次递增 1，跨控件互不影响，不随复位重置。
 *          例如启动按钮连按 5 次会依次得到 event=1,2,3,4,5。
 *
 * 业务代码应忽略此字段、仅按 widget_id 分发，除非将来需要做
 * 双击/连点/长按一类的高级交互，才用它做计数或差值判断。
 *
 * 下面的 PRESS/RELEASE 常量语义已废弃，保留仅为兼容旧调用点。
 * ============================================================ */

typedef enum {
    SCREEN_TOUCH_PRESS   = 0x0000,  /* 已废弃, 勿用于过滤 */
    SCREEN_TOUCH_RELEASE = 0x0001,  /* 已废弃, 勿用于过滤 */
} screen_touch_event_t;

/* ============================================================
 * 触控上报数据结构
 * ============================================================ */

typedef struct {
    uint16_t page_id;       /* 当前页面 ID */
    uint16_t widget_id;     /* 被触摸的控件 ID */
    uint16_t x;             /* 触摸 X 坐标 */
    uint16_t y;             /* 触摸 Y 坐标 */
    uint16_t event;         /* 事件类型 (screen_touch_event_t) */
} screen_touch_data_t;

/**
 * @brief 触控事件回调函数类型
 *
 * 当屏幕上报触控事件时调用此回调。
 * 注意: 回调在解析任务上下文中执行，不要做耗时操作。
 *
 * @param data 触控数据
 */
typedef void (*screen_touch_cb_t)(const screen_touch_data_t *data);

/**
 * @brief 页面更新回调函数类型
 *
 * 当 MCU 调用 screen_query_page() 查询当前页面后, 屏幕返回响应时触发此回调。
 * 注意: 回调在解析任务上下文中执行，不要做耗时操作。
 *
 * @param current_page 屏幕上报的当前页面 ID
 */
typedef void (*screen_page_update_cb_t)(uint16_t current_page);

/* ============================================================
 * API 接口
 * ============================================================ */

/**
 * @brief 初始化串口屏协议层
 *
 * 初始化帧接收解析状态机。不初始化 UART 硬件。
 *
 * @return ESP_OK 成功
 */
esp_err_t screen_protocol_init(void);

/**
 * @brief 切换屏幕页面
 *
 * @param page_id 目标页面 ID (从0开始)
 * @return ESP_OK 成功
 */
esp_err_t screen_switch_page(uint16_t page_id);

/**
 * @brief 设置控件显示文本
 *
 * @param widget_id 控件 ID (在 sHMI 上位机中分配)
 * @param text      UTF-8 字符串 (以 '\0' 结尾)
 * @return ESP_OK 成功
 */
esp_err_t screen_set_text(uint16_t widget_id, const char *text);

/**
 * @brief 设置控件数值 (进度条/滑块/弧形/图片索引)
 *
 * @param widget_id 控件 ID
 * @param value     数值
 * @return ESP_OK 成功
 */
esp_err_t screen_set_val(uint16_t widget_id, uint16_t value);

/**
 * @brief 设置控件显隐
 *
 * @param widget_id 控件 ID
 * @param hidden    true=隐藏, false=显示
 * @return ESP_OK 成功
 */
esp_err_t screen_set_hidden(uint16_t widget_id, bool hidden);

/**
 * @brief 设置背光亮度
 *
 * @param brightness 亮度值 (0~255, 0=最暗, 255=最亮)
 * @return ESP_OK 成功
 */
esp_err_t screen_set_backlight(uint8_t brightness);

/**
 * @brief 设置控件前景色
 *
 * @param widget_id 控件 ID
 * @param color     颜色值 (32位 ARGB)
 * @return ESP_OK 成功
 */
esp_err_t screen_set_font_color(uint16_t widget_id, uint32_t color);

/**
 * @brief 注册触控事件回调
 *
 * @param cb 回调函数指针，传 NULL 可取消
 * @return ESP_OK 成功
 */
esp_err_t screen_register_touch_callback(screen_touch_cb_t cb);

/**
 * @brief 查询屏幕当前所在页面
 *
 * 发送指令 EE 00 02 06 [CRC] FF FC FF FF, 屏幕返回
 * EE 00 02 06 [page_H page_L] [CRC] FF FC FF FF。
 * 响应解析后通过 screen_register_page_callback 注册的回调通知上层。
 *
 * 调用前必须已通过 bus_scheduler 获取 485 总线。
 *
 * @return ESP_OK 发送成功 (注意: 不代表已收到响应, 响应是异步的)
 */
esp_err_t screen_query_page(void);

/**
 * @brief 注册页面查询响应回调
 *
 * @param cb 回调函数指针，传 NULL 可取消
 * @return ESP_OK 成功
 */
esp_err_t screen_register_page_callback(screen_page_update_cb_t cb);

/**
 * @brief 喂入从 UART 接收到的原始字节
 *
 * 由 screen_service 的接收任务调用，将 UART 收到的数据交给协议层解析。
 * 内部状态机会识别触控帧，解析完成后调用注册的回调。
 *
 * @param data  接收到的字节数组
 * @param len   字节数
 */
void screen_feed_rx(const uint8_t *data, uint16_t len);

#endif /* SCREEN_PROTOCOL_H */
