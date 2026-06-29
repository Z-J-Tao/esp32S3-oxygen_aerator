/**
 * @file cabin_fsm.h
 * @brief 氧舱状态机 — 系统核心模块
 *
 * 【概述】
 *   氧舱状态机是整个微压增氧控制系统的"大脑"，统一编排所有执行器动作。
 *   它维护两个层级的状态：
 *     1. 操作阶段 (cabin_phase_t): 描述氧舱从开机到关机的完整流程
 *     2. 氧舱状态 (cabin_state_t): 描述运行阶段中具体的气压控制状态
 *
 * 【操作阶段流程】
 *   初始化 → 自检 → 校准 → 消毒 → 设定(压力/时间/速度) → 待关门 → 运行 → 急停/减压 → 结束
 *
 * 【状态转移示例 — 正常使用流程】
 *   停机
 *    ↓ FSM_EVT_START (用户按启动)
 *   开机升压(<5KPa)
 *    ↓ 压力达到 5KPa
 *   开机升压(>=5KPa)
 *    ↓ 压力达到目标值
 *   保压 1~10KPa / 保压 >10~20KPa / 保压 >20~30KPa (根据目标压力)
 *    ↓ 保压定时到期 FSM_EVT_TIMER_EXPIRED
 *   正常泄压
 *    ↓ 压力降到 5KPa
 *   泄压至5KPa
 *    ↓ 压力降到 0
 *   停机
 *
 * 【状态转移示例 — 应急流程】
 *   任意状态
 *    ↓ FSM_EVT_ESTOP (应急按钮 GPIO3)
 *   加速泄压 → 泄压至5KPa → 停机
 *
 * 【每个状态对应的执行器设置】
 *   参见操作逻辑图中的状态机表：
 *   "O" = 供电(开), "×" = 停止供电(关), "×/O" = 根据条件决定
 *
 *   停机状态:
 *     制氧组=关, 增氧泵=关, 进气阀=全关, 换气阀=全关,
 *     泄气阀正常流量=开, 大流量泄气阀=关, 风扇=关
 *
 *   开机升压(<5KPa):
 *     制氧组=开, 增氧泵=开, 进气阀=关(低压不进气), 换气阀=关,
 *     泄气阀=关, 风扇=关
 *
 *   开机升压(>=5KPa):
 *     制氧组=开, 增氧泵=开, 进气阀=按速度选开1个, 换气阀=关,
 *     泄气阀=关, 风扇=关
 *
 *   保压 1~10KPa:
 *     制氧组=开, 增氧泵=开, 进气阀=关, 换气阀6#=开(其他关),
 *     泄气阀=关, 风扇=关
 *
 *   （其他状态参见状态机表）
 */
#ifndef CABIN_FSM_H
#define CABIN_FSM_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== 氧舱状态定义 ==================== */

/**
 * 12 态状态机（与主控端/小程序规范对齐）
 *
 * 状态名称字符串（用于 MQTT status 上报）由 cabin_fsm_get_state_name() 返回：
 *   "shutdown" / "pressurizing_init" / "pressurizing" /
 *   "holding_low" / "holding_mid" / "holding_high" /
 *   "depressurizing" / "depressurizing_fast" / "depressurizing_final" /
 *   "overtime" / "fault" / "paused"
 */
typedef enum {
    CABIN_STATE_SHUTDOWN = 0,           // 停机：所有设备关，泄气阀正常流量开
    CABIN_STATE_PRESSURIZING_INIT,      // 开机升压 (<5KPa)：制氧+增氧开，进气关
    CABIN_STATE_PRESSURIZING,           // 开机升压 (>=5KPa)：制氧+增氧开，进气按速度选开
    CABIN_STATE_PAUSED,                 // 暂停：制氧+增氧保持，进气/换气关
    CABIN_STATE_HOLDING_LOW,            // 保压 1~10KPa：换气阀6#开
    CABIN_STATE_HOLDING_MID,            // 保压 >10~20KPa：换气阀7#开
    CABIN_STATE_HOLDING_HIGH,           // 保压 >20~30KPa：换气阀8#开
    CABIN_STATE_DEPRESSURIZING,         // 正常泄压：泄气阀正常流量开
    CABIN_STATE_DEPRESSURIZING_FAST,    // 加速泄压：泄气阀正常+大流量全开
    CABIN_STATE_DEPRESSURIZING_FINAL,   // 泄压至5KPa以下：泄气阀全开直到 <=5KPa
    CABIN_STATE_OVERTIME,               // 超时：保压定时到期但用户未开门
    CABIN_STATE_FAULT,                  // 故障：异常保护，所有设备关
    CABIN_STATE_MAX
} cabin_state_t;

/* ==================== 操作阶段定义 ==================== */

typedef enum {
    CABIN_PHASE_INIT = 0,           // 初始化：系统上电初始化
    CABIN_PHASE_SELFCHECK,          // 自检：检查传感器/执行器是否正常
    CABIN_PHASE_CALIBRATE,          // 校准：传感器零点校准
    CABIN_PHASE_DISINFECT,          // 消毒：运行消毒程序（定时）
    CABIN_PHASE_SETTING,            // 设定：用户设置目标压力、时间、速度
    CABIN_PHASE_WAIT_DOOR,          // 待关门：等待舱门关闭信号
    CABIN_PHASE_RUNNING,            // 运行中：升压 → 保压 → 泄压
    CABIN_PHASE_ESTOP,              // 急停：应急按钮触发，快速泄压
    CABIN_PHASE_DEPRESSURIZE,       // 减压：正常流程的泄压阶段
    CABIN_PHASE_FINISHED,           // 结束：一次使用完毕
} cabin_phase_t;

/* ==================== 进气速度 ==================== */

typedef enum {
    INTAKE_SPEED_LOW = 0,           // 低速：CH2 (3#进) 开
    INTAKE_SPEED_MED,               // 中速：CH3 (4#进) 开
    INTAKE_SPEED_FAST,              // 快速：CH4 (5#进) 开
} intake_speed_t;

/* ==================== 运行参数 ==================== */

/**
 * @brief 氧舱运行参数（在 SETTING 阶段由用户通过串口屏/语音/MQTT 设置）
 */
typedef struct {
    float    target_pressure_kpa;   // 目标保压压力 (KPa)，范围 1~30
    uint32_t hold_time_min;         // 保压时间 (分钟)
    intake_speed_t intake_speed;    // 进气速度选择
} cabin_run_params_t;

/* ==================== 执行器通道定义 ==================== */

/*
 * PCA9685 通道号 → 设备映射
 * 与原理图 PCB P2 和状态机表完全对应
 */
#define CH_OXYGEN_GEN       0       // 1#氧  制氧组             AC220V
#define CH_OXYGEN_PUMP      1       // 2#增  增氧泵             AC220V
#define CH_INTAKE_LOW       2       // 3#进  进气阀-低速         DC12V
#define CH_INTAKE_MED       3       // 4#进  进气阀-中速         DC12V
#define CH_INTAKE_FAST      4       // 5#进  进气阀-快速         DC12V
#define CH_VENT_10          5       // 6#换  换气阀 1~10KPa      DC12V
#define CH_VENT_20          6       // 7#换  换气阀 >10~20KPa    DC12V
#define CH_VENT_30          7       // 8#换  换气阀 >20~30KPa    DC12V
#define CH_DEFLATE_NORM_O   8       // 9#排  泄气阀-正常流量(开)  DC12V
#define CH_DEFLATE_NORM_C   9       // 10#闭 泄气阀-正常流量(闭)  DC12V
#define CH_DEFLATE_HIGH_O   10      // 11#排 泄气阀-大流量(开)    DC12V
#define CH_DEFLATE_HIGH_C   11      // 12#闭 泄气阀-大流量(闭)    DC12V
#define CH_EXHAUST_FAN      12      // 13#扇 管道排气风扇         DC12V
#define CH_LIGHT            13      // 14#灯 手动控制灯光         AC220V
#define CH_AC               14      // 15#冷 手动控制空调         AC220V
// CH15 备用

/* ==================== 状态机接口 ==================== */

/**
 * @brief 初始化状态机
 *
 * 创建事件队列，设置初始状态为 SHUTDOWN。
 * 必须在 actuator_service_init() 之后调用。
 *
 * @return ESP_OK 成功
 */
esp_err_t cabin_fsm_init(void);

/**
 * @brief 启动状态机服务任务
 *
 * 创建 FreeRTOS 任务 "cabin_fsm_task"，优先级建议 6（高于传感器和屏幕）。
 * 主循环：等待事件 → 执行状态转移 → 设置执行器 → 播报语音。
 *
 * @return ESP_OK 任务创建成功
 */
esp_err_t cabin_fsm_start(void);

/**
 * @brief 获取当前氧舱状态
 * @return 当前 cabin_state_t
 */
cabin_state_t cabin_fsm_get_state(void);

/**
 * @brief 获取当前氧舱状态的字符串名称（用于 MQTT status 上报）
 *
 * 返回值示例："shutdown" / "pressurizing_init" / "holding_mid" 等。
 * 返回的指针指向静态常量字符串，调用方无需释放。
 *
 * @return 状态名称字符串
 */
const char *cabin_fsm_get_state_name(void);

/**
 * @brief 获取当前操作阶段
 * @return 当前 cabin_phase_t
 */
cabin_phase_t cabin_fsm_get_phase(void);

/**
 * @brief 获取当前操作阶段的字符串名称
 * @return 阶段名称字符串（如 "init" / "running" / "depressurize" 等）
 */
const char *cabin_fsm_get_phase_name(void);

/**
 * @brief 获取当前状态的中文名称（给串口屏显示用）
 *
 * 与 cabin_fsm_get_state_name() 的区别:
 *   - get_state_name(): 英文 snake_case, 对齐 MQTT 上报字段
 *   - get_state_name_cn(): 中文, 对齐串口屏 WID_FSM_STATE 控件的显示需求
 *
 * @return 中文状态名, 如 "停机" / "升压" / "保压(中)" / "故障"
 */
const char *cabin_fsm_get_state_name_cn(void);

/**
 * @brief 获取当前阶段的进度百分比 (0~100), 供主页压力进度条显示
 *
 * 按当前状态分段计算:
 *   - PRESSURIZING_INIT / PRESSURIZING: 以 current_pressure / target_pressure 表征升压进度
 *   - HOLDING_LOW/MID/HIGH:           以 elapsed_min / hold_time_min 表征保压进度
 *   - DEPRESSURIZING*:                以 (target - current) / target 表征泄压进度
 *   - SHUTDOWN / FAULT / PAUSED / OVERTIME: 返回 0
 *
 * 结果永远 clamp 到 [0, 100], 不会越界。
 * 读不到传感器数据时返回 0。
 *
 * @return 0~100
 */
uint8_t cabin_fsm_get_progress_pct(void);

/**
 * @brief 运行时信息（供 MQTT state_detail 使用）
 */
typedef struct {
    float    target_pressure_kpa;   /* 目标保压压力 (KPa) */
    uint32_t hold_time_min;         /* 设定的保压时间 (分钟) */
    uint32_t elapsed_min;           /* 已运行/已保压分钟数 */
    uint32_t remaining_min;         /* 保压剩余分钟数 */
} cabin_run_info_t;

/**
 * @brief 获取当前运行信息（线程安全）
 *
 * 用于 MQTT status 上报中的 state_detail 字段。
 *
 * @param info [out] 运行信息
 * @return ESP_OK 成功
 */
esp_err_t cabin_fsm_get_run_info(cabin_run_info_t *info);

/**
 * @brief 设置运行参数
 *
 * 仅在 CABIN_PHASE_SETTING 阶段有效，其他阶段调用返回错误。
 *
 * @param params 运行参数指针
 * @return ESP_OK 成功，ESP_ERR_INVALID_STATE 不在设定阶段
 */
esp_err_t cabin_fsm_set_params(const cabin_run_params_t *params);

/* ==================== 状态机事件 ==================== */

/**
 * @brief 状态机事件类型
 *
 * 各模块通过 cabin_fsm_send_event() 向状态机发送事件，
 * 状态机在主循环中取出事件并执行对应的状态转移。
 */
typedef enum {
    FSM_EVT_START,              // 启动运行（来自：串口屏/语音/MQTT cmd: "start"）
    FSM_EVT_PAUSE,              // 暂停（来自：串口屏/语音/MQTT cmd: "pause"）
    FSM_EVT_RESUME,             // 恢复（来自：串口屏/语音/MQTT cmd: "resume"）
    FSM_EVT_ESTOP,              // 急停（来自：应急按钮 GPIO3 / MQTT cmd: "emergency_stop"）
    FSM_EVT_DOOR_CLOSED,        // 舱门已关（来自：门磁传感器/手动确认）
    FSM_EVT_DOOR_OPENED,        // 舱门已开
    FSM_EVT_PRESSURE_UPDATE,    // 压力数据更新（来自：sensor_service 每轮轮询后）
    FSM_EVT_TIMER_EXPIRED,      // 保压定时到期（来自：内部定时器）
    FSM_EVT_FAULT,              // 故障（来自：自检失败/传感器离线/执行器异常）
    FSM_EVT_RESET,              // 复位回停机（来自：故障恢复/手动复位 / MQTT cmd: "stop"）
    FSM_EVT_VENTILATE,          // 换气（来自：MQTT cmd: "ventilate"）
    FSM_EVT_DISINFECT,          // 消毒（来自：MQTT cmd: "disinfect"）
} fsm_event_t;

/**
 * @brief 向状态机发送事件
 *
 * 线程安全，可从任意任务/ISR 中调用。
 * FSM_EVT_ESTOP 为最高优先级事件，发送到队列头部。
 *
 * @param event 事件类型
 * @return ESP_OK 成功入队，ESP_ERR_TIMEOUT 队列满
 */
esp_err_t cabin_fsm_send_event(fsm_event_t event);

#endif // CABIN_FSM_H
