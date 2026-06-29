/**
 * @file voice_module.h
 * @brief ASR PRO 离线语音模块驱动
 *
 * 【硬件连接】
 *   - ESP32 UART2 TX (GPIO11) → ASR PRO PA3 (RX)
 *   - ESP32 UART2 RX (GPIO12) ← ASR PRO PA2 (TX)
 *   - ASR PRO 由外部 +5V 供电
 *   - 全双工通信，独立 UART，不占用 485 总线
 *
 * 【模块说明】
 *   ASR PRO 是一款离线语音识别模块，内置麦克风和功放接口。
 *   语音词条和播报内容在 ASR PRO 的开发工具中离线配置烧录，
 *   ESP32 不需要做语音处理，只需通过串口收发指令编号。
 *
 * 【通信协议】
 *   需根据 ASR PRO 的串口协议手册确认，通常为简单的指令帧：
 *     - ESP32 → ASR PRO: 发送播报指令 ID，模块播放对应预录音频
 *     - ASR PRO → ESP32: 用户说出唤醒词+指令后，模块返回识别结果 ID
 *
 * 【播报场景】
 *   - 报警播报：温度过高、氧浓度异常、压力异常等
 *   - 状态提示：开始加压、保压中、开始泄压、使用结束等
 *   - 应急播报：紧急停止、请保持镇定等
 *
 * 【语音指令场景】
 *   - "开始" / "停止" / "暂停" → 控制氧舱状态机
 *   - "开灯" / "关灯" → 手动控制灯光
 *   - "打开空调" / "关闭空调" → 手动控制空调
 */
#ifndef VOICE_MODULE_H
#define VOICE_MODULE_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief 初始化 ASR PRO 语音模块驱动
 *
 * 依赖：bsp_uart2_voice_init() 必须已完成
 *
 * 初始化内容：
 *   - 创建 UART2 接收任务（后台持续监听 ASR PRO 返回的识别结果）
 *   - （可选）发送测试指令确认模块在线
 *
 * @return ESP_OK 成功
 */
esp_err_t voice_module_init(void);

/**
 * @brief 播报指定语音
 *
 * 向 ASR PRO 发送播报指令，模块播放对应编号的预录音频。
 * 播报 ID 在 ASR PRO 的开发工具中定义。
 *
 * @param voice_id 预录制语音编号（0-255，具体编号查 ASR PRO 配置）
 * @return ESP_OK 成功，ESP_ERR_TIMEOUT 发送超时
 */
esp_err_t voice_play(uint8_t voice_id);

/**
 * @brief 语音识别结果回调函数类型
 *
 * 当 ASR PRO 识别到用户语音指令后，通过 UART 返回识别结果 ID，
 * 驱动解析后调用此回调通知上层 voice_service。
 *
 * @param cmd_id 识别到的语音指令编号（由 ASR PRO 配置决定）
 */
typedef void (*voice_recognize_cb_t)(uint8_t cmd_id);

/**
 * @brief 注册语音识别结果回调
 * @param cb 回调函数指针，传 NULL 可取消
 * @return ESP_OK 成功
 */
esp_err_t voice_register_recognize_callback(voice_recognize_cb_t cb);

#endif // VOICE_MODULE_H
