/**
 * @file cabin_fsm.c
 * @brief 氧舱状态机实现
 *
 * 【核心数据结构】
 *   - s_state:  当前氧舱状态（12种之一）
 *   - s_phase:  当前操作阶段（10种之一）
 *   - s_params: 用户设定的运行参数
 *   - s_evt_queue: 事件队列（FreeRTOS Queue）
 *
 * 【状态转移实现方式】
 *   建议使用"状态转移表"或"switch-case"两种方式之一：
 *
 *   方式 A: 转移表（推荐，清晰且易维护）
 *     定义 transition_table[CABIN_STATE_MAX][FSM_EVT_MAX] = { ... };
 *     每个表项包含：目标状态 + 动作函数指针
 *
 *   方式 B: switch-case（简单直接）——当前实现使用此方式
 *     switch (s_state) {
 *         case CABIN_STATE_SHUTDOWN:
 *             if (event == FSM_EVT_START) { → 切到 PRESSURIZING_INIT }
 *             break;
 *         ...
 *     }
 *
 * 【压力驱动状态转移 — FSM_EVT_PRESSURE_UPDATE】
 *   sensor_service 每 2 秒读取气压传感器并发送此事件，FSM 据此自动切换状态：
 *
 *   升压阶段:
 *     PRESSURIZING_INIT + 压力 >= 5KPa    → PRESSURIZING（开启进气阀）
 *     PRESSURIZING      + 压力 >= 目标值  → HOLDING_LOW/MID/HIGH（进入保压）
 *
 *   保压阶段（动态区间切换）:
 *     HOLDING_xxx + 实际压力变化            → 切换到对应的 HOLDING_LOW/MID/HIGH
 *     HOLDING_xxx + FSM_EVT_TIMER_EXPIRED  → DEPRESSURIZING（保压时间到期）
 *
 *   泄压阶段:
 *     DEPRESSURIZING      + 压力 <= 5KPa  → DEPRESSURIZING_FINAL（收尾泄压）
 *     DEPRESSURIZING_FAST + 压力 <= 5KPa  → DEPRESSURIZING_FINAL
 *     DEPRESSURIZING_FINAL + 压力 ≈ 0     → SHUTDOWN（结束）
 *
 * 【ESTOP 与 FAULT 的区分】
 *   ESTOP (应急按钮): → DEPRESSURIZING_FAST（先安全泄压再停机）
 *   FAULT (严重故障): → FAULT（立即全关，安全模式）
 *
 * 【保压定时器】
 *   使用 esp_timer one-shot 模式：
 *   - 进入保压时启动（hold_time_min 分钟）
 *   - 暂停/恢复时暂停/恢复计时
 *   - 到期发送 FSM_EVT_TIMER_EXPIRED → 开始泄压
 *
 * 【状态进入动作】
 *   每次进入新状态时，需要调用 actuator_set_all() 设置执行器。
 *   各状态对应的 bitmap 可预定义为常量数组：
 *
 *   static const uint16_t state_actuator_map[CABIN_STATE_MAX] = {
 *       [CABIN_STATE_SHUTDOWN]              = (1<<CH_DEFLATE_NORM_O) | (1<<CH_DEFLATE_NORM_C),
 *       [CABIN_STATE_PRESSURIZING_INIT]     = (1<<CH_OXYGEN_GEN) | (1<<CH_OXYGEN_PUMP),
 *       [CABIN_STATE_PRESSURIZING]          = (1<<CH_OXYGEN_GEN) | (1<<CH_OXYGEN_PUMP) | (进气阀根据速度),
 *       [CABIN_STATE_HOLDING_LOW]           = (1<<CH_OXYGEN_GEN) | (1<<CH_OXYGEN_PUMP) | (1<<CH_VENT_10),
 *       [CABIN_STATE_HOLDING_MID]           = (1<<CH_OXYGEN_GEN) | (1<<CH_OXYGEN_PUMP) | (1<<CH_VENT_20),
 *       [CABIN_STATE_HOLDING_HIGH]          = (1<<CH_OXYGEN_GEN) | (1<<CH_OXYGEN_PUMP) | (1<<CH_VENT_30),
 *       [CABIN_STATE_DEPRESSURIZING]        = (1<<CH_DEFLATE_NORM_O) | (1<<CH_DEFLATE_NORM_C),
 *       [CABIN_STATE_DEPRESSURIZING_FAST]   = (1<<CH_DEFLATE_NORM_O) | (1<<CH_DEFLATE_NORM_C) |
 *                                             (1<<CH_DEFLATE_HIGH_O) | (1<<CH_DEFLATE_HIGH_C),
 *       [CABIN_STATE_FAULT]                 = 0,  // 全关
 *       // ...
 *   };
 */
#include "cabin_fsm.h"
#include "common/app_config.h"
#include "common/app_events.h"
#include "services/actuator_service.h"
#include "services/voice_service.h"
#include "services/sensor_service.h"
#include "bsp/bsp_gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "cabin_fsm";

/* ── 压力阈值常量 (KPa) ── */
#define PRESSURE_INIT_THRESHOLD     5.0f    /* <5KPa: 升压初期, >=5KPa: 升压(开进气阀) */
#define PRESSURE_DEPRESS_FINAL      5.0f    /* 泄压收尾阈值：压力降到此值以下切 FINAL */
#define PRESSURE_SHUTDOWN_THRESHOLD 0.5f    /* 泄压结束阈值：压力接近零即回到停机 */

static cabin_state_t      s_state = CABIN_STATE_SHUTDOWN;
static cabin_phase_t      s_phase = CABIN_PHASE_INIT;
static cabin_run_params_t s_params = {0};
static QueueHandle_t      s_evt_queue = NULL;
static TaskHandle_t       s_fsm_task_handle = NULL;
static cabin_state_t      s_state_before_pause = CABIN_STATE_SHUTDOWN;

/* ── 保压定时器 ── */
static esp_timer_handle_t s_hold_timer = NULL;
static int64_t            s_hold_remaining_us = 0;  /* 暂停时保存的剩余时间 (微秒) */
static int64_t            s_hold_start_us = 0;      /* 当前定时器启动时刻 (esp_timer_get_time) */
static int64_t            s_hold_period_us = 0;     /* 当前定时器设定周期 (微秒) */

/* ── 状态→执行器 bitmap 映射表 ──
 * 每次进入新状态时，通过 actuator_set_all() 一次性设置所有通道。
 * PRESSURIZING 状态的进气阀需根据 s_params.intake_speed 动态叠加，见 fsm_handle_event()。
 */
static const uint16_t state_actuator_map[CABIN_STATE_MAX] = {
    [CABIN_STATE_SHUTDOWN]              = (1 << CH_DEFLATE_NORM_O) | (1 << CH_DEFLATE_NORM_C),
    [CABIN_STATE_PRESSURIZING_INIT]     = (1 << CH_OXYGEN_GEN) | (1 << CH_OXYGEN_PUMP),
    [CABIN_STATE_PRESSURIZING]          = (1 << CH_OXYGEN_GEN) | (1 << CH_OXYGEN_PUMP),
    [CABIN_STATE_PAUSED]                = (1 << CH_OXYGEN_GEN) | (1 << CH_OXYGEN_PUMP),
    [CABIN_STATE_HOLDING_LOW]           = (1 << CH_OXYGEN_GEN) | (1 << CH_OXYGEN_PUMP) | (1 << CH_VENT_10),
    [CABIN_STATE_HOLDING_MID]           = (1 << CH_OXYGEN_GEN) | (1 << CH_OXYGEN_PUMP) | (1 << CH_VENT_20),
    [CABIN_STATE_HOLDING_HIGH]          = (1 << CH_OXYGEN_GEN) | (1 << CH_OXYGEN_PUMP) | (1 << CH_VENT_30),
    [CABIN_STATE_DEPRESSURIZING]        = (1 << CH_DEFLATE_NORM_O) | (1 << CH_DEFLATE_NORM_C),
    [CABIN_STATE_DEPRESSURIZING_FAST]   = (1 << CH_DEFLATE_NORM_O) | (1 << CH_DEFLATE_NORM_C) |
                                          (1 << CH_DEFLATE_HIGH_O) | (1 << CH_DEFLATE_HIGH_C),
    [CABIN_STATE_DEPRESSURIZING_FINAL]  = (1 << CH_DEFLATE_NORM_O) | (1 << CH_DEFLATE_NORM_C) |
                                          (1 << CH_DEFLATE_HIGH_O) | (1 << CH_DEFLATE_HIGH_C),
    [CABIN_STATE_OVERTIME]              = 0,
    [CABIN_STATE_FAULT]                 = 0,    /* 全关 */
};

/* ── 状态名称（中文日志用）── */
static const char *state_names_cn[CABIN_STATE_MAX] = {
    [CABIN_STATE_SHUTDOWN]              = "停机",
    [CABIN_STATE_PRESSURIZING_INIT]     = "升压初期",
    [CABIN_STATE_PRESSURIZING]          = "升压",
    [CABIN_STATE_PAUSED]                = "暂停",
    [CABIN_STATE_HOLDING_LOW]           = "保压(低)",
    [CABIN_STATE_HOLDING_MID]           = "保压(中)",
    [CABIN_STATE_HOLDING_HIGH]          = "保压(高)",
    [CABIN_STATE_DEPRESSURIZING]        = "正常泄压",
    [CABIN_STATE_DEPRESSURIZING_FAST]   = "加速泄压",
    [CABIN_STATE_DEPRESSURIZING_FINAL]  = "泄压收尾",
    [CABIN_STATE_OVERTIME]              = "超时",
    [CABIN_STATE_FAULT]                 = "故障",
};

/* ── 事件名称（日志用）── */
#define FSM_EVT_COUNT 12
static const char *event_names[FSM_EVT_COUNT] = {
    [FSM_EVT_START]             = "START",
    [FSM_EVT_PAUSE]             = "PAUSE",
    [FSM_EVT_RESUME]            = "RESUME",
    [FSM_EVT_ESTOP]             = "ESTOP",
    [FSM_EVT_DOOR_CLOSED]       = "DOOR_CLOSED",
    [FSM_EVT_DOOR_OPENED]       = "DOOR_OPENED",
    [FSM_EVT_PRESSURE_UPDATE]   = "PRESSURE_UPDATE",
    [FSM_EVT_TIMER_EXPIRED]     = "TIMER_EXPIRED",
    [FSM_EVT_FAULT]             = "FAULT",
    [FSM_EVT_RESET]             = "RESET",
    [FSM_EVT_VENTILATE]         = "VENTILATE",
    [FSM_EVT_DISINFECT]         = "DISINFECT",
};

/**
 * @brief 保压定时器回调 — 在定时器任务上下文中运行
 *
 * 保压时间到期时，向 FSM 事件队列发送 TIMER_EXPIRED 事件。
 */
static void hold_timer_callback(void *arg)
{
    (void)arg;
    fsm_event_t evt = FSM_EVT_TIMER_EXPIRED;
    if (s_evt_queue != NULL) {
        xQueueSendToBack(s_evt_queue, &evt, 0);
    }
    ESP_LOGI(TAG, "保压定时器到期 → FSM_EVT_TIMER_EXPIRED");
}

/* ── 保压定时器辅助函数 ── */

/** 启动保压定时器（从头开始计时） */
static void hold_timer_start(uint32_t hold_time_min)
{
    if (s_hold_timer == NULL || hold_time_min == 0) return;

    s_hold_period_us = (int64_t)hold_time_min * 60LL * 1000000LL;
    s_hold_remaining_us = s_hold_period_us;
    s_hold_start_us = esp_timer_get_time();

    esp_timer_stop(s_hold_timer);   /* 确保之前的不残留 */
    esp_err_t ret = esp_timer_start_once(s_hold_timer, s_hold_period_us);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "保压定时器启动: %"PRIu32" 分钟 (%lld us)",
                 hold_time_min, (long long)s_hold_period_us);
    } else {
        ESP_LOGE(TAG, "保压定时器启动失败: %s", esp_err_to_name(ret));
    }
}

/** 暂停保压定时器（记录剩余时间） */
static void hold_timer_pause(void)
{
    if (s_hold_timer == NULL) return;

    int64_t elapsed = esp_timer_get_time() - s_hold_start_us;
    s_hold_remaining_us = s_hold_period_us - elapsed;
    if (s_hold_remaining_us < 0) s_hold_remaining_us = 0;

    esp_timer_stop(s_hold_timer);
    ESP_LOGI(TAG, "保压定时器暂停 (剩余 %lld s)", (long long)(s_hold_remaining_us / 1000000LL));
}

/** 恢复保压定时器（用剩余时间继续） */
static void hold_timer_resume(void)
{
    if (s_hold_timer == NULL || s_hold_remaining_us <= 0) return;

    s_hold_period_us = s_hold_remaining_us;
    s_hold_start_us = esp_timer_get_time();

    esp_timer_stop(s_hold_timer);
    esp_err_t ret = esp_timer_start_once(s_hold_timer, s_hold_remaining_us);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "保压定时器恢复 (剩余 %lld s)", (long long)(s_hold_remaining_us / 1000000LL));
    }
}

/** 停止保压定时器 */
static void hold_timer_stop(void)
{
    if (s_hold_timer == NULL) return;
    esp_timer_stop(s_hold_timer);
    s_hold_remaining_us = 0;
    s_hold_period_us = 0;
}

/* ── 辅助函数：根据目标压力确定保压区间 ── */
static cabin_state_t get_holding_state_for_pressure(float target_kpa)
{
    if (target_kpa <= 10.0f) return CABIN_STATE_HOLDING_LOW;
    if (target_kpa <= 20.0f) return CABIN_STATE_HOLDING_MID;
    return CABIN_STATE_HOLDING_HIGH;
}

/* ── 辅助函数：判断当前压力是否属于某保压区间 ── */
static cabin_state_t get_holding_state_by_current(float current_kpa)
{
    if (current_kpa <= 10.0f) return CABIN_STATE_HOLDING_LOW;
    if (current_kpa <= 20.0f) return CABIN_STATE_HOLDING_MID;
    return CABIN_STATE_HOLDING_HIGH;
}

/**
 * @brief 应急按钮回调 — 在 ISR 上下文中调用
 *
 * 将 FSM_EVT_ESTOP 发送到事件队列头部，确保最高优先级处理。
 * 同时设置全局事件组的 EVT_EMERGENCY 位，供其他模块感知。
 */
static void IRAM_ATTR on_emergency_btn(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    fsm_event_t evt = FSM_EVT_ESTOP;

    /* ESTOP 发到队列头部，优先处理 */
    if (s_evt_queue != NULL) {
        xQueueSendToFrontFromISR(s_evt_queue, &evt, &xHigherPriorityTaskWoken);
    }

    /* 通知全局事件组，供 screen_service / rule_engine 等模块感知 */
    if (g_system_events != NULL) {
        xEventGroupSetBitsFromISR(g_system_events, EVT_EMERGENCY,
                                  &xHigherPriorityTaskWoken);
    }

    /* 视觉反馈：点亮调试 LED 表示急停已触发 */
    bsp_led_set(true);

    /* 如果唤醒了更高优先级任务，触发上下文切换 */
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t cabin_fsm_init(void)
{
    /* 1. 创建事件队列（深度 16，足够缓冲多个事件） */
    s_evt_queue = xQueueCreate(16, sizeof(fsm_event_t));
    if (s_evt_queue == NULL) {
        ESP_LOGE(TAG, "创建事件队列失败");
        return ESP_FAIL;
    }

    /* 2. 设置初始状态 */
    s_state = CABIN_STATE_SHUTDOWN;
    s_phase = CABIN_PHASE_INIT;

    /* 3. 创建保压定时器（one-shot 模式，到期后发送 FSM_EVT_TIMER_EXPIRED） */
    const esp_timer_create_args_t timer_args = {
        .callback = hold_timer_callback,
        .arg      = NULL,
        .name     = "hold_timer",
    };
    esp_err_t terr = esp_timer_create(&timer_args, &s_hold_timer);
    if (terr != ESP_OK) {
        ESP_LOGE(TAG, "创建保压定时器失败: %s", esp_err_to_name(terr));
        /* 非致命：定时器失败不阻止启动，但保压计时将不可用 */
    }

    /* 4. 注册应急按钮回调（GPIO3 中断 → 发送 FSM_EVT_ESTOP） */
    bsp_register_emergency_callback(on_emergency_btn);

    ESP_LOGI(TAG, "状态机初始化完成 (队列深度=16, 初始状态=SHUTDOWN)");
    return ESP_OK;
}

/* ── 辅助函数：更新操作阶段 ── */
static void update_phase(cabin_state_t state)
{
    switch (state) {
        case CABIN_STATE_SHUTDOWN:
            s_phase = CABIN_PHASE_INIT;
            break;
        case CABIN_STATE_PRESSURIZING_INIT:
        case CABIN_STATE_PRESSURIZING:
        case CABIN_STATE_PAUSED:
        case CABIN_STATE_HOLDING_LOW:
        case CABIN_STATE_HOLDING_MID:
        case CABIN_STATE_HOLDING_HIGH:
        case CABIN_STATE_OVERTIME:
            s_phase = CABIN_PHASE_RUNNING;
            break;
        case CABIN_STATE_DEPRESSURIZING:
        case CABIN_STATE_DEPRESSURIZING_FAST:
        case CABIN_STATE_DEPRESSURIZING_FINAL:
            s_phase = CABIN_PHASE_DEPRESSURIZE;
            break;
        case CABIN_STATE_FAULT:
            s_phase = CABIN_PHASE_ESTOP;
            break;
        default:
            break;
    }
}

/* ── 辅助函数：根据状态转移决定播报内容 ── */
static uint8_t get_transition_voice_id(cabin_state_t old_state, cabin_state_t new_state)
{
    if (new_state == CABIN_STATE_FAULT) {
        return VOICE_EMERGENCY;             /* 0x20 "紧急停机" */
    }
    if (new_state == CABIN_STATE_PRESSURIZING_INIT) {
        return VOICE_STATE_PRESSUP;         /* 0x02 "开始升压" */
    }
    if (new_state == CABIN_STATE_HOLDING_LOW ||
        new_state == CABIN_STATE_HOLDING_MID ||
        new_state == CABIN_STATE_HOLDING_HIGH) {
        return VOICE_STATE_HOLDING;         /* 0x03 "保压运行中" */
    }
    if (new_state == CABIN_STATE_DEPRESSURIZING ||
        new_state == CABIN_STATE_DEPRESSURIZING_FAST) {
        return VOICE_STATE_DEPRESSURE;      /* 0x04 "开始泄压" */
    }
    /* 泄压完成 → 停机：播报"本次使用结束" */
    if (new_state == CABIN_STATE_SHUTDOWN &&
        (old_state == CABIN_STATE_DEPRESSURIZING ||
         old_state == CABIN_STATE_DEPRESSURIZING_FAST ||
         old_state == CABIN_STATE_DEPRESSURIZING_FINAL)) {
        return VOICE_STATE_DONE;            /* 0x05 "本次使用结束" */
    }
    return 0;   /* 不播报 */
}

/**
 * @brief 状态转移核心逻辑
 *
 * ESTOP: 从任意状态 → 加速泄压 (DEPRESSURIZING_FAST)
 * FAULT: 从任意状态 → 故障安全模式 (FAULT, 全部关闭)
 * PRESSURE_UPDATE: 根据当前压力值自动驱动状态切换
 * 其他事件: 根据当前状态分发处理
 */
static void fsm_handle_event(fsm_event_t event)
{
    cabin_state_t old_state = s_state;
    cabin_state_t new_state = old_state;

    /* ── ESTOP: 从任意状态进入加速泄压 ──
     * 与 FAULT 不同，ESTOP 是"安全泄压"：打开大流量泄气阀尽快降压后再停机。
     * FAULT 是"立即全关"：传感器故障等严重异常，直接断电保护。
     *
     * 反馈兜底: 当 ESTOP 在已经停机/泄压中的状态下按下时, 没有状态可切,
     * 但仍要给用户反馈 (LED 已被 ISR 点亮, 这里补一次语音 + 日志确认),
     * 避免用户认为按钮失灵反复按 → 反而压垮总线/语音队列。
     */
    if (event == FSM_EVT_ESTOP) {
        if (old_state == CABIN_STATE_SHUTDOWN ||
            old_state == CABIN_STATE_DEPRESSURIZING_FAST ||
            old_state == CABIN_STATE_DEPRESSURIZING_FINAL) {
            ESP_LOGW(TAG, "ESTOP 按下: 已在 [%s] 无需切换, 仅反馈确认",
                     state_names_cn[old_state]);
            voice_service_play(VOICE_EMERGENCY);
            return;
        }
        hold_timer_stop();
        new_state = CABIN_STATE_DEPRESSURIZING_FAST;
        goto apply;
    }

    /* ── FAULT: 从任意状态直接进入安全模式（全部关闭）── */
    if (event == FSM_EVT_FAULT) {
        if (old_state != CABIN_STATE_FAULT) {
            hold_timer_stop();
            new_state = CABIN_STATE_FAULT;
        }
        goto apply;
    }

    /* ── 按当前状态分发事件 ── */
    switch (old_state) {

    case CABIN_STATE_SHUTDOWN:
        if (event == FSM_EVT_START) {
            new_state = CABIN_STATE_PRESSURIZING_INIT;
        }
        break;

    case CABIN_STATE_PRESSURIZING_INIT:
        if (event == FSM_EVT_PAUSE) {
            new_state = CABIN_STATE_PAUSED;
        } else if (event == FSM_EVT_RESET) {
            new_state = CABIN_STATE_DEPRESSURIZING;
        } else if (event == FSM_EVT_PRESSURE_UPDATE) {
            /* 读取当前压力 */
            sensor_data_t sd;
            if (sensor_service_get_data(&sd) == ESP_OK) {
                if (sd.pressure_kpa >= s_params.target_pressure_kpa) {
                    /* 直接达到目标 → 进入保压 */
                    new_state = get_holding_state_for_pressure(s_params.target_pressure_kpa);
                } else if (sd.pressure_kpa >= PRESSURE_INIT_THRESHOLD) {
                    /* 超过 5KPa → 开启进气阀加速升压 */
                    new_state = CABIN_STATE_PRESSURIZING;
                }
                /* 否则保持 PRESSURIZING_INIT */
            }
        }
        break;

    case CABIN_STATE_PRESSURIZING:
        if (event == FSM_EVT_PAUSE) {
            new_state = CABIN_STATE_PAUSED;
        } else if (event == FSM_EVT_RESET) {
            new_state = CABIN_STATE_DEPRESSURIZING;
        } else if (event == FSM_EVT_PRESSURE_UPDATE) {
            sensor_data_t sd;
            if (sensor_service_get_data(&sd) == ESP_OK) {
                if (sd.pressure_kpa >= s_params.target_pressure_kpa) {
                    /* 达到目标压力 → 进入保压 */
                    new_state = get_holding_state_for_pressure(s_params.target_pressure_kpa);
                }
                /* 如果压力意外跌回 <5KPa，仍保持 PRESSURIZING（进气阀已开）*/
            }
        }
        break;

    case CABIN_STATE_PAUSED:
        if (event == FSM_EVT_RESUME) {
            new_state = s_state_before_pause;
        } else if (event == FSM_EVT_RESET) {
            hold_timer_stop();
            new_state = CABIN_STATE_DEPRESSURIZING;
        }
        break;

    case CABIN_STATE_HOLDING_LOW:
    case CABIN_STATE_HOLDING_MID:
    case CABIN_STATE_HOLDING_HIGH:
        if (event == FSM_EVT_PAUSE) {
            new_state = CABIN_STATE_PAUSED;
        } else if (event == FSM_EVT_TIMER_EXPIRED) {
            /* 保压时间到期 → 正常泄压 */
            new_state = CABIN_STATE_DEPRESSURIZING;
        } else if (event == FSM_EVT_RESET) {
            hold_timer_stop();
            new_state = CABIN_STATE_DEPRESSURIZING;
        } else if (event == FSM_EVT_PRESSURE_UPDATE) {
            /* 保压期间根据实际压力动态调整换气阀区间 */
            sensor_data_t sd;
            if (sensor_service_get_data(&sd) == ESP_OK) {
                cabin_state_t correct_hold = get_holding_state_by_current(sd.pressure_kpa);
                if (correct_hold != old_state) {
                    new_state = correct_hold;
                    /* 注意：保压区间切换不重置定时器 */
                }
            }
        }
        break;

    case CABIN_STATE_DEPRESSURIZING:
        if (event == FSM_EVT_RESET) {
            new_state = CABIN_STATE_SHUTDOWN;
        } else if (event == FSM_EVT_PRESSURE_UPDATE) {
            sensor_data_t sd;
            if (sensor_service_get_data(&sd) == ESP_OK) {
                if (sd.pressure_kpa <= PRESSURE_SHUTDOWN_THRESHOLD) {
                    /* 压力≈0 → 停机 */
                    new_state = CABIN_STATE_SHUTDOWN;
                } else if (sd.pressure_kpa <= PRESSURE_DEPRESS_FINAL) {
                    /* 压力 ≤5KPa → 泄压收尾 */
                    new_state = CABIN_STATE_DEPRESSURIZING_FINAL;
                }
            }
        }
        break;

    case CABIN_STATE_DEPRESSURIZING_FAST:
        if (event == FSM_EVT_RESET) {
            new_state = CABIN_STATE_SHUTDOWN;
        } else if (event == FSM_EVT_PRESSURE_UPDATE) {
            sensor_data_t sd;
            if (sensor_service_get_data(&sd) == ESP_OK) {
                if (sd.pressure_kpa <= PRESSURE_SHUTDOWN_THRESHOLD) {
                    new_state = CABIN_STATE_SHUTDOWN;
                } else if (sd.pressure_kpa <= PRESSURE_DEPRESS_FINAL) {
                    new_state = CABIN_STATE_DEPRESSURIZING_FINAL;
                }
            }
        }
        break;

    case CABIN_STATE_DEPRESSURIZING_FINAL:
        if (event == FSM_EVT_RESET) {
            new_state = CABIN_STATE_SHUTDOWN;
        } else if (event == FSM_EVT_PRESSURE_UPDATE) {
            sensor_data_t sd;
            if (sensor_service_get_data(&sd) == ESP_OK) {
                if (sd.pressure_kpa <= PRESSURE_SHUTDOWN_THRESHOLD) {
                    new_state = CABIN_STATE_SHUTDOWN;
                }
            }
        }
        break;

    case CABIN_STATE_OVERTIME:
        if (event == FSM_EVT_RESET) {
            new_state = CABIN_STATE_DEPRESSURIZING;
        }
        break;

    case CABIN_STATE_FAULT:
        if (event == FSM_EVT_RESET) {
            new_state = CABIN_STATE_SHUTDOWN;
        }
        /* FAULT 状态下忽略除 RESET 外的所有事件 */
        break;

    default:
        break;
    }

apply:
    /* ── 状态未变化，忽略 ── */
    if (new_state == old_state) {
        return;
    }

    /* ── 1. 暂停前保存状态（用于 RESUME 恢复）── */
    if (new_state == CABIN_STATE_PAUSED) {
        s_state_before_pause = old_state;
    }

    /* ── 2. 设置执行器 ── */
    if (new_state == CABIN_STATE_FAULT) {
        /* 故障：单条 I2C 命令全关（最快路径）*/
        actuator_all_off();
    } else {
        uint16_t bitmap = state_actuator_map[new_state];
        /* PRESSURIZING 状态：根据进气速度动态叠加阀门通道 */
        if (new_state == CABIN_STATE_PRESSURIZING) {
            switch (s_params.intake_speed) {
                case INTAKE_SPEED_LOW:  bitmap |= (1 << CH_INTAKE_LOW);  break;
                case INTAKE_SPEED_MED:  bitmap |= (1 << CH_INTAKE_MED);  break;
                case INTAKE_SPEED_FAST: bitmap |= (1 << CH_INTAKE_FAST); break;
            }
        }
        actuator_set_all(bitmap);
    }

    /* ── 3. 保压定时器管理 ── */
    /* 进入保压区间：启动定时器 */
    if ((new_state == CABIN_STATE_HOLDING_LOW ||
         new_state == CABIN_STATE_HOLDING_MID ||
         new_state == CABIN_STATE_HOLDING_HIGH) &&
        old_state != CABIN_STATE_HOLDING_LOW &&
        old_state != CABIN_STATE_HOLDING_MID &&
        old_state != CABIN_STATE_HOLDING_HIGH &&
        old_state != CABIN_STATE_PAUSED) {
        /* 从非保压/非暂停状态首次进入保压 → 全新启动 */
        hold_timer_start(s_params.hold_time_min);
    }
    /* 暂停 → 保压：恢复定时器 */
    if ((new_state == CABIN_STATE_HOLDING_LOW ||
         new_state == CABIN_STATE_HOLDING_MID ||
         new_state == CABIN_STATE_HOLDING_HIGH) &&
        old_state == CABIN_STATE_PAUSED) {
        hold_timer_resume();
    }
    /* 保压 → 暂停：暂停定时器 */
    if (new_state == CABIN_STATE_PAUSED &&
        (old_state == CABIN_STATE_HOLDING_LOW ||
         old_state == CABIN_STATE_HOLDING_MID ||
         old_state == CABIN_STATE_HOLDING_HIGH)) {
        hold_timer_pause();
    }
    /* 离开保压进入泄压/故障：停止定时器 */
    if (new_state == CABIN_STATE_DEPRESSURIZING ||
        new_state == CABIN_STATE_DEPRESSURIZING_FAST ||
        new_state == CABIN_STATE_DEPRESSURIZING_FINAL ||
        new_state == CABIN_STATE_FAULT ||
        new_state == CABIN_STATE_SHUTDOWN) {
        hold_timer_stop();
    }

    /* ── 4. 语音播报 ── */
    uint8_t voice_id = get_transition_voice_id(old_state, new_state);
    if (voice_id != 0) {
        voice_service_play(voice_id);
    }

    /* ── 5. 更新状态和阶段 ── */
    s_state = new_state;
    update_phase(new_state);

    /* ── 6. LED 控制 ── */
    if (new_state == CABIN_STATE_FAULT) {
        bsp_led_set(true);      /* 故障亮灯（ISR 中可能已点亮，重复设置无害） */
    } else if (old_state == CABIN_STATE_FAULT) {
        bsp_led_set(false);     /* 故障恢复灭灯 */
    }

    ESP_LOGW(TAG, "状态转移: %s → %s (事件: %s)",
             state_names_cn[old_state], state_names_cn[new_state],
             (event < FSM_EVT_COUNT) ? event_names[event] : "?");
}

/**
 * @brief FSM 主任务 — 优先级 6（系统最高）
 *
 * 永久阻塞等待事件队列，收到事件后调用 fsm_handle_event() 处理状态转移。
 */
static void cabin_fsm_task(void *arg)
{
    fsm_event_t event;
    ESP_LOGI(TAG, "状态机任务启动 (优先级=%d)", uxTaskPriorityGet(NULL));

    while (1) {
        if (xQueueReceive(s_evt_queue, &event, portMAX_DELAY) == pdTRUE) {
            fsm_handle_event(event);
        }
    }
}

esp_err_t cabin_fsm_start(void)
{
    /* 栈 6144: fsm_handle_event 在状态转移时会级联调用
     *   actuator_set_all (内部 16 次 PCA9685 I2C 写 + LOG)
     *   voice_service_play (队列发送 + LOG)
     *   多处 ESP_LOGI/W (单次约 1.5KB)
     * 启动→升压→保压→泄压几步压力告急, 4096 偏紧, 提到 6144 留余量 */
    BaseType_t ret = xTaskCreate(cabin_fsm_task, "cabin_fsm", 6144, NULL, 6, &s_fsm_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 cabin_fsm 任务失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "状态机任务已启动");
    return ESP_OK;
}

cabin_state_t cabin_fsm_get_state(void)
{
    return s_state;
}

cabin_phase_t cabin_fsm_get_phase(void)
{
    return s_phase;
}

const char *cabin_fsm_get_phase_name(void)
{
    static const char *phase_names[] = {
        [CABIN_PHASE_INIT]          = "init",
        [CABIN_PHASE_SELFCHECK]     = "selfcheck",
        [CABIN_PHASE_CALIBRATE]     = "calibrate",
        [CABIN_PHASE_DISINFECT]     = "disinfect",
        [CABIN_PHASE_SETTING]       = "setting",
        [CABIN_PHASE_WAIT_DOOR]     = "wait_door",
        [CABIN_PHASE_RUNNING]       = "running",
        [CABIN_PHASE_ESTOP]         = "estop",
        [CABIN_PHASE_DEPRESSURIZE]  = "depressurize",
        [CABIN_PHASE_FINISHED]      = "finished",
    };
    cabin_phase_t ph = s_phase;
    if (ph > CABIN_PHASE_FINISHED) return "unknown";
    return phase_names[ph];
}

esp_err_t cabin_fsm_get_run_info(cabin_run_info_t *info)
{
    if (info == NULL) return ESP_ERR_INVALID_ARG;

    info->target_pressure_kpa = s_params.target_pressure_kpa;
    info->hold_time_min       = s_params.hold_time_min;

    /* 计算保压已用/剩余时间 */
    if ((s_state == CABIN_STATE_HOLDING_LOW ||
         s_state == CABIN_STATE_HOLDING_MID ||
         s_state == CABIN_STATE_HOLDING_HIGH) &&
        s_hold_period_us > 0) {
        int64_t elapsed_us = esp_timer_get_time() - s_hold_start_us;
        int64_t remaining_us = s_hold_period_us - elapsed_us;
        if (remaining_us < 0) remaining_us = 0;
        info->elapsed_min   = (uint32_t)(elapsed_us / 60000000LL);
        info->remaining_min = (uint32_t)(remaining_us / 60000000LL);
    } else if (s_state == CABIN_STATE_PAUSED && s_hold_remaining_us > 0) {
        /* 暂停状态：用保存的剩余时间 */
        uint32_t total = s_params.hold_time_min;
        uint32_t remain = (uint32_t)(s_hold_remaining_us / 60000000LL);
        info->remaining_min = remain;
        info->elapsed_min   = (total > remain) ? (total - remain) : 0;
    } else {
        info->elapsed_min   = 0;
        info->remaining_min = 0;
    }

    return ESP_OK;
}

const char *cabin_fsm_get_state_name(void)
{
    /* 与 MQTT status 上报中的 "state" 字段对应，与主控端/小程序规范对齐 */
    static const char *state_names[CABIN_STATE_MAX] = {
        [CABIN_STATE_SHUTDOWN]              = "shutdown",
        [CABIN_STATE_PRESSURIZING_INIT]     = "pressurizing_init",
        [CABIN_STATE_PRESSURIZING]          = "pressurizing",
        [CABIN_STATE_PAUSED]                = "paused",
        [CABIN_STATE_HOLDING_LOW]           = "holding_low",
        [CABIN_STATE_HOLDING_MID]           = "holding_mid",
        [CABIN_STATE_HOLDING_HIGH]          = "holding_high",
        [CABIN_STATE_DEPRESSURIZING]        = "depressurizing",
        [CABIN_STATE_DEPRESSURIZING_FAST]   = "depressurizing_fast",
        [CABIN_STATE_DEPRESSURIZING_FINAL]  = "depressurizing_final",
        [CABIN_STATE_OVERTIME]              = "overtime",
        [CABIN_STATE_FAULT]                 = "fault",
    };
    cabin_state_t st = s_state;
    if (st >= CABIN_STATE_MAX) return "unknown";
    return state_names[st];
}

const char *cabin_fsm_get_state_name_cn(void)
{
    /* 直接复用文件顶部的 state_names_cn[] 表, 日志和屏幕显示同一份数据源 */
    cabin_state_t st = s_state;
    if (st >= CABIN_STATE_MAX) return "未知";
    return state_names_cn[st];
}

uint8_t cabin_fsm_get_progress_pct(void)
{
    cabin_state_t st = s_state;

    /* 停机/故障/暂停/超时: 无进度概念 */
    if (st == CABIN_STATE_SHUTDOWN ||
        st == CABIN_STATE_FAULT    ||
        st == CABIN_STATE_PAUSED   ||
        st == CABIN_STATE_OVERTIME) {
        return 0;
    }

    sensor_data_t sd;
    if (sensor_service_get_data(&sd) != ESP_OK) {
        return 0;
    }

    float pct = 0.0f;
    float target = s_params.target_pressure_kpa;

    if (st == CABIN_STATE_PRESSURIZING_INIT || st == CABIN_STATE_PRESSURIZING) {
        /* 升压阶段: 压力占目标的比例 */
        if (target > 0.01f) {
            pct = sd.pressure_kpa / target * 100.0f;
        }
    } else if (st == CABIN_STATE_HOLDING_LOW ||
               st == CABIN_STATE_HOLDING_MID ||
               st == CABIN_STATE_HOLDING_HIGH) {
        /* 保压阶段: 按时间进度 */
        if (s_params.hold_time_min > 0 && s_hold_period_us > 0) {
            int64_t elapsed_us = esp_timer_get_time() - s_hold_start_us;
            if (elapsed_us < 0) elapsed_us = 0;
            pct = (float)elapsed_us / (float)s_hold_period_us * 100.0f;
        }
    } else if (st == CABIN_STATE_DEPRESSURIZING ||
               st == CABIN_STATE_DEPRESSURIZING_FAST ||
               st == CABIN_STATE_DEPRESSURIZING_FINAL) {
        /* 泄压阶段: 进度随压力下降单调上升 (目标压力→100%, 0KPa→完成) */
        if (target > 0.01f) {
            pct = (target - sd.pressure_kpa) / target * 100.0f;
        }
    }

    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint8_t)(pct + 0.5f);
}

esp_err_t cabin_fsm_set_params(const cabin_run_params_t *params)
{
    if (params == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 仅 SETTING 阶段允许修改参数 */
    if (s_phase != CABIN_PHASE_SETTING && s_phase != CABIN_PHASE_INIT) {
        ESP_LOGW(TAG, "当前阶段(%d)不允许修改参数", s_phase);
        return ESP_ERR_INVALID_STATE;
    }

    /* 范围检查 */
    if (params->target_pressure_kpa < 1.0f || params->target_pressure_kpa > 30.0f) {
        ESP_LOGW(TAG, "目标压力 %.1f KPa 超出范围 (1~30)", params->target_pressure_kpa);
        return ESP_ERR_INVALID_ARG;
    }

    s_params = *params;
    ESP_LOGI(TAG, "运行参数已设置: 目标压力=%.1fKPa, 保压时间=%lu分钟, 进气速度=%d",
             s_params.target_pressure_kpa, s_params.hold_time_min, s_params.intake_speed);
    return ESP_OK;
}

esp_err_t cabin_fsm_send_event(fsm_event_t event)
{
    if (s_evt_queue == NULL) {
        ESP_LOGE(TAG, "事件队列未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ret;
    if (event == FSM_EVT_ESTOP) {
        /* 急停事件：发到队列头部，优先处理 */
        ret = xQueueSendToFront(s_evt_queue, &event, pdMS_TO_TICKS(100));
    } else {
        /* 普通事件：发到队列尾部 */
        ret = xQueueSendToBack(s_evt_queue, &event, pdMS_TO_TICKS(100));
    }

    if (ret != pdTRUE) {
        ESP_LOGW(TAG, "事件入队失败 (event=%d), 队列可能已满", event);
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}
