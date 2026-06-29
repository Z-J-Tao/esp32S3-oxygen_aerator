/**
 * @file voice_service.h
 * @brief 语音交互业务服务
 *
 * 【职责】
 *   - 监听 ASR PRO 返回的语音识别结果，分发到对应模块
 *   - 提供统一的语音播报接口，供报警、状态提示等场景调用
 *
 * 【与 voice_module 的关系】
 *   voice_module （驱动层）：UART2 收发、帧解析
 *   voice_service（服务层）：业务分发，决定"识别了做什么"和"什么时候播报"
 *
 * 【语音识别指令处理】（cmd_id 由 ASR PRO 离线配置决定）
 *   - "开始" → cabin_fsm_send_event(FSM_EVT_START)
 *   - "暂停" → cabin_fsm_send_event(FSM_EVT_PAUSE)
 *   - "停止" → cabin_fsm_send_event(FSM_EVT_RESET)
 *   - "开灯" → actuator_set(CH_LIGHT, true)
 *   - "关灯" → actuator_set(CH_LIGHT, false)
 *   - "打开空调" → actuator_set(CH_AC, true)
 *   - "关闭空调" → actuator_set(CH_AC, false)
 *
 * 【语音播报场景】
 *   - 报警：rule_engine 检测到阈值超限 → voice_service_play(VOICE_ALARM_xxx)
 *   - 状态：cabin_fsm 状态切换时 → voice_service_play(VOICE_STATE_xxx)
 *   - 应急：应急按钮触发 → voice_service_play(VOICE_EMERGENCY)
 */
#ifndef VOICE_SERVICE_H
#define VOICE_SERVICE_H

#include "esp_err.h"
#include <stdint.h>

/* ── 播报 voice_id 定义（与 ASR PRO 端 WAV 文件编号对应）── */

/* 状态提示 */
#define VOICE_STATE_BOOT        0x01    // "系统启动"
#define VOICE_STATE_PRESSUP     0x02    // "开始升压"
#define VOICE_STATE_HOLDING     0x03    // "保压运行中"
#define VOICE_STATE_DEPRESSURE  0x04    // "开始泄压"
#define VOICE_STATE_DONE        0x05    // "本次使用结束"

/* 报警播报 */
#define VOICE_ALARM_TEMP        0x10    // "温度异常报警"
#define VOICE_ALARM_PRESSURE    0x11    // "压力异常报警"
#define VOICE_ALARM_OXYGEN      0x12    // "氧浓度异常报警"（三级共用同一条语音）

/* 应急播报 */
#define VOICE_EMERGENCY         0x20    // "紧急停机"

/**
 * @brief 启动语音服务
 *
 * 注册语音识别回调，创建指令分发逻辑。
 * voice_module 的接收任务在 voice_module_init() 中已创建，
 * 此处只注册回调来处理识别结果。
 *
 * @return ESP_OK 成功
 */
esp_err_t voice_service_start(void);

/**
 * @brief 触发语音播报
 *
 * 对 voice_play() 的封装，增加播报队列管理（防止多个播报同时触发时冲突）。
 *
 * @param voice_id 预录制语音编号（在 ASR PRO 工具中定义）
 * @return ESP_OK 成功加入播报队列
 */
esp_err_t voice_service_play(uint8_t voice_id);

#endif // VOICE_SERVICE_H
