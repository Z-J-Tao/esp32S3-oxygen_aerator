/**
 * @file modbus_master.h
 * @brief Modbus RTU 主站驱动
 *
 * 通过 UART1 (RS485) 与 Modbus 从站传感器通信。
 *
 * 【通信链路】
 *   ESP32 GPIO17(TX) → TTL转485模块 DI → RS485 总线 A+/B-
 *   ESP32 GPIO18(RX) ← TTL转485模块 RO ← RS485 总线 A+/B-
 *   TTL转485模块内置自动 DE/RE 方向控制，无需额外引脚
 *
 * 【协议说明】
 *   - Modbus RTU 帧格式：[地址1B][功能码1B][数据NB][CRC16 2B]
 *   - 帧间隔：至少 3.5 个字符时间（9600bps 下约 4ms）
 *   - CRC16 校验（低字节在前）
 *
 * 【常用功能码】
 *   - 0x03: 读保持寄存器（Read Holding Registers）
 *   - 0x04: 读输入寄存器（Read Input Registers）
 *   - 0x06: 写单个保持寄存器（Write Single Register）
 *
 * 【注意事项】
 *   - 485 总线与串口屏共用，必须通过 bus_scheduler 获取总线后再操作
 *   - 建议实现方式：使用 ESP-IDF 的 freemodbus 组件，或自行实现 RTU 帧收发
 */
#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief 初始化 Modbus RTU 主站
 *
 * 依赖：bsp_uart1_rs485_init() 必须已完成
 *
 * 初始化内容：
 *   - 配置 Modbus 主站参数（超时、重试次数等）
 *   - 如果使用 freemodbus 组件，则初始化 MB master 实例
 *   - 如果自行实现，则准备发送/接收缓冲区
 *
 * @return ESP_OK 成功
 */
esp_err_t modbus_master_init(void);

/**
 * @brief 读取从站保持寄存器（功能码 0x03）
 *
 * 用途：读取传感器的配置参数或实时数据
 *
 * @param slave_addr 从站地址 (1-247)
 * @param reg_addr   起始寄存器地址
 * @param reg_count  要读取的寄存器数量
 * @param data       [out] 读取结果缓冲区，调用者分配，至少 reg_count 个 uint16_t
 * @return ESP_OK 成功，ESP_ERR_TIMEOUT 超时无响应，ESP_ERR_INVALID_CRC CRC 校验失败
 */
esp_err_t modbus_master_read_holding(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_count, uint16_t *data);

/**
 * @brief 读取从站输入寄存器（功能码 0x04）
 *
 * 用途：读取传感器的实时测量值（温湿度、气压、氧气浓度等）
 *
 * @param slave_addr 从站地址 (1-247)
 * @param reg_addr   起始寄存器地址
 * @param reg_count  要读取的寄存器数量
 * @param data       [out] 读取结果缓冲区
 * @return ESP_OK 成功
 */
esp_err_t modbus_master_read_input(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_count, uint16_t *data);

/**
 * @brief 写单个保持寄存器（功能码 0x06）
 *
 * 用途：修改传感器配置（如校准参数、地址修改等）
 *
 * @param slave_addr 从站地址 (1-247)
 * @param reg_addr   目标寄存器地址
 * @param value      要写入的值
 * @return ESP_OK 成功
 */
esp_err_t modbus_master_write_single(uint8_t slave_addr, uint16_t reg_addr, uint16_t value);

/**
 * @brief "非 Modbus 数据"回调函数类型
 *
 * 当 Modbus 收发过程中遇到不符合 Modbus 格式的字节时, 通过此回调把原始字节
 * 传给上层, 让上层的其他协议解析器 (如串口屏 screen_feed_rx) 有机会识别。
 * 场景:
 *   1. 发送请求前 UART RX FIFO 有残留 (非本次应答, 通常是屏幕触控帧或查询响应)
 *   2. 响应帧 CRC 校验失败 / 帧格式异常
 *
 * @param data 收到的原始字节
 * @param len  字节数
 */
typedef void (*modbus_rx_fallback_cb_t)(const uint8_t *data, uint16_t len);

/**
 * @brief 注册"非 Modbus 数据"回调
 *
 * 典型用法: sensor_service 初始化时把 screen_feed_rx 注册进来,
 * 这样 485 总线共用时屏幕发出的帧不会被 Modbus 吞掉。
 *
 * @param cb 回调函数指针, 传 NULL 可取消
 * @return ESP_OK 成功
 */
esp_err_t modbus_master_register_rx_fallback(modbus_rx_fallback_cb_t cb);

#endif // MODBUS_MASTER_H
