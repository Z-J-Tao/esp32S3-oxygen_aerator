/**
 * @file screen_service.c
 * @brief 串口屏业务服务实现
 *
 * 【架构说明】
 *   本模块创建两个 FreeRTOS 任务:
 *
 *   1) screen_tx_task — 数据推送任务
 *      - 每 500ms 或收到 EVT_SENSOR_DATA_READY 事件时触发
 *      - 获取 485 总线 → 调用 screen_set_text() 推送数据 → 释放总线
 *
 *   2) screen_rx_task — 接收解析任务
 *      - 持续从 UART1 读取字节，喂给 screen_feed_rx() 进行帧解析
 *      - 解析到触控帧时通过回调通知本模块
 *      - 注意: 仅在总线空闲期间能收到屏幕上报 (屏幕发送也需��总线)
 *
 * 【触控事件处理】
 *   触控回调 on_touch_event() 根据 widget_id 分发:
 *   - 启动/暂停/停止 → cabin_fsm_send_event()
 *   - 灯光/空调 → actuator_set()
 *   - 紧急停止 → cabin_fsm_send_event(FSM_EVT_EMERGENCY)
 *   - 参数设定 → 暂存，等用户点击"保存"后批量写入
 *
 * 【控件 ID 映射】
 *   控件 ID 待用户提供 sHMI 上位机配置后填入 screen_widgets.h
 *   当前使用 TODO 占位符
 */
#include "screen_service.h"
#include "common/app_config.h"
#include "common/app_events.h"
#include "services/bus_scheduler.h"
#include "services/timer_service.h"
#include "services/sensor_service.h"
#include "services/cabin_fsm.h"
#include "services/actuator_service.h"
#include "drivers/screen_protocol.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <stdio.h>
#include <time.h>
#include <limits.h>
#include <stdbool.h>

static const char *TAG = "screen_svc";

/* ============================================================
 * 配置
 * ============================================================ */

#define SCREEN_TX_TASK_STACK    4096
#define SCREEN_TX_TASK_PRIO     4
#define SCREEN_RX_TASK_STACK    3072
#define SCREEN_RX_TASK_PRIO     4

#define SCREEN_REFRESH_MS       500     /* 数据推送周期 (ms) */
#define SCREEN_BUS_TIMEOUT_MS   1000    /* 总线获取超时 (ms) */
#define SCREEN_RX_TIMEOUT_MS    20      /* UART 接收超时 (ms) */

#define PAGE_QUERY_DELAY_US     100000  /* 导航按钮按下后 100ms 发查询, 让屏幕先切完页 */

/* ============================================================
 * 页面状态机
 *
 * 每个页面控件 ID 独立编号 (例如 widget_id=17 在主页是温度数值,
 * 在执行器页是 CH13 绿灯指示), 所以推送前必须先知道当前在哪页。
 *
 * 状态同步三条通道:
 *   1. 触控帧 on_touch_event → 若为导航按钮, 启动 100ms 定时器后主动查询
 *   2. 查询响应 on_page_update → 权威更新 s_current_page (回调到 tx 任务)
 *   3. 启动时 screen_service_start 强制切到主页并同步
 * ============================================================ */

typedef enum {
    PAGE_MAIN       = 0,    /* 第 0 页: 主监控 (温湿度/压力/氧/状态/进度/按钮) */
    PAGE_SETTING    = 1,    /* 第 1 页: 参数设置 (目标压力/保压时间/进气速度) */
    PAGE_ACTUATOR   = 2,    /* 第 2 页: 执行器状态 (15 路继电器指示灯) */
    PAGE_ALARM      = 3,    /* 第 3 页: 报警记录 */
    PAGE_NETWORK    = 4,    /* 第 4 页: 网络设置 (WiFi/MQTT 状态) */
    PAGE_SYSTEM     = 5,    /* 第 5 页: 系统信息 (固件版本/MCU) */
    PAGE_MAX
} screen_page_t;

static volatile uint16_t  s_current_page    = PAGE_MAIN;
static TaskHandle_t       s_tx_task_handle  = NULL;
static esp_timer_handle_t s_query_timer     = NULL;
static volatile bool      s_query_pending   = false;

/* 页面切换后, 屏幕端控件会恢复到 HMI 设计时的默认状态 (尤其是显隐状态等不被
 * 屏幕缓存的属性)。MCU 侧的 last_X 缓存仍是旧值, 导致"状态已同步"的误判而
 * 跳过推送。此标志位在切页时置 true, 各 push_X_page() 首段检查并重置本地缓存,
 * 强制所有字段再推一次, 确保屏幕内容与实际状态一致。*/
static volatile bool      s_page_force_refresh = false;

/* ============================================================
 * 控件 ID 定义 (等待 sHMI 上位机配置后更新)
 * TODO: 用户提供控件 ID 映射后替换这些占位值
 *       下方 WID_TIME 为用户实机确认值，其余仍是占位
 * ============================================================ */

/* Page 0: 主监控页 — 显示控件 */
#define WID_TIME            0x0004  /* 顶栏时间 HH:MM (实机确认) */
#define WID_TEMP_VALUE      0x0011  /* 温度数值 label (实机确认控件17) */
#define WID_HUMI_VALUE      0x0012  /* 湿度数值 label (实机确认控件18) */
#define WID_PRESS_VALUE     0x0013  /* 压力数值 label (实机确认控件19) */
#define WID_O2_VALUE        0x0014  /* 氧浓度数值 label (实机确认控件20) */
#define WID_FSM_STATE       0x0010  /* 当前阶段文本 label */
#define WID_REMAIN_TIME     0x002E  /* 剩余时间 label (实机确认控件46, 格式 "HH:MM" 与顶栏时间一致, 每分钟刷新) */
#define WID_PRESS_BAR       0x0018  /* 压力进度条 (实机确认控件24) */
#define WID_PRESS_BAR_PCT   0x0019  /* 压力进度百分比数字 (实机确认控件25, 与进度条联动, 不带 % 后缀) */
#define WID_ALARM_TEXT      0x0020  /* 报警滚动文本 */
#define WID_WIFI_ICON       0x0007  /* WiFi 状态图标 (当前测试用控件7) */
#define WID_MQTT_ICON       0x0008  /* MQTT "未连接"提示控件 (连接后隐藏) */

/* Page 0: 主监控页 — 触控按钮
 * 注意: event 字段是"每控件独立的按下次数计数器" (实测验证),
 *       跨控件互不影响，分发逻辑只依赖 widget_id, 不要依赖 event */
#define WID_BTN_START       0x001C  /* 启动按钮 (实机确认控件28) */
#define WID_BTN_PAUSE       0x001D  /* 暂停按钮 (实机确认控件29) */
#define WID_BTN_STOP        0x001E  /* 停止按钮 (实机确认控件30) */
#define WID_BTN_ESTOP       0x0021  /* 紧急停止按钮 (实机确认控件33) */
#define WID_BTN_DISINFECT   0x0028  /* 消毒按钮 (实机确认控件40) */
#define WID_BTN_LIGHT       0x0103  /* 灯光按钮 (占位, 待实机确认) */
#define WID_BTN_AC          0x0104  /* 空调按钮 (占位, 待实机确认) */

/* Page 1: 参数设置页 */
#define WID_SET_PRESSURE    0x0200  /* 目标压力显示 */
#define WID_SET_HOLD_TIME   0x0201  /* 保压时间显示 */
#define WID_SET_SPEED       0x0202  /* 进气速度选择 */
#define WID_BTN_SAVE        0x0210  /* 保存按钮 */
#define WID_BTN_DEFAULT     0x0211  /* 恢复默认按钮 */

/* Page 0: 跳转到子页的导航按钮 (全部在主页底部工具栏, 实机图确认) */
#define WID_NAV_SETTING     0x0022  /* 控件34 → 第1页 参数设置 */
#define WID_NAV_ACTUATOR    0x0023  /* 控件35 → 第2页 执行器状态 */
#define WID_NAV_ALARM       0x0024  /* 控件36 → 第3页 报警记录 */
#define WID_NAV_NETWORK     0x0025  /* 控件37 → 第4页 网络设置 */
#define WID_NAV_SYSTEM      0x0026  /* 控件38 → 第5页 系统信息 */

/* 子页 → 主页: "返回" 按钮, 所有子页(1~5)都是 widget_id=2 (实机确认) */
#define WID_NAV_BACK        0x0002

/**
 * @brief 判断当前触控 (page, widget) 是否为导航按钮
 *
 * 导航按钮按下后屏幕会内部切页, MCU 需要延时发查询指令确认新页面。
 *
 * @return true 是导航按钮 / false 普通业务按钮
 */
static bool is_nav_widget(uint16_t page, uint16_t wid)
{
    if (page == PAGE_MAIN) {
        return (wid == WID_NAV_SETTING  || wid == WID_NAV_ACTUATOR ||
                wid == WID_NAV_ALARM    || wid == WID_NAV_NETWORK  ||
                wid == WID_NAV_SYSTEM);
    }
    /* 第 1~5 页的"返回"按钮 */
    if (page >= PAGE_SETTING && page < PAGE_MAX) {
        return (wid == WID_NAV_BACK);
    }
    return false;
}

/* ============================================================
 * 页面查询定时器 / 回调
 * ============================================================ */

/**
 * @brief 查询定时器回调 — 在导航按钮按下 100ms 后触发
 *
 * 只设置 pending 标志 + 唤醒 tx 任务, 真正的 bus_acquire + 发送在 tx 任务里做,
 * 避免在定时器回调里持有 485 总线引发阻塞。
 */
static void query_timer_cb(void *arg)
{
    s_query_pending = true;
    if (s_tx_task_handle) {
        xTaskNotifyGive(s_tx_task_handle);
    }
}

/**
 * @brief 页面查询响应回调 — 屏幕返回当前页面时触发
 *
 * 在 rx 任务上下文中执行, 这里只更新状态并唤醒 tx 任务。
 */
static void on_page_update(uint16_t current_page)
{
    if (current_page >= PAGE_MAX) {
        ESP_LOGW(TAG, "查询响应页面越界: %d, 忽略", current_page);
        return;
    }
    if (current_page == s_current_page) {
        ESP_LOGD(TAG, "页面未变化: %d", current_page);
        return;
    }
    ESP_LOGI(TAG, "页面切换确认: %d → %d", s_current_page, current_page);
    s_current_page = current_page;
    s_page_force_refresh = true;   /* 新页面需要强制全量刷新一次 */
    if (s_tx_task_handle) {
        xTaskNotifyGive(s_tx_task_handle);   /* 立即刷新新页面 */
    }
}

/* ============================================================
 * 触控事件处理
 * ============================================================ */

static void on_touch_event(const screen_touch_data_t *data)
{
    /* event 是此 widget 的按下次数计数器(1,2,3,...), 业务不关心,
     * 只按 widget_id 分发; 打印 event 仅用于调试和将来可能的连点检测 */
    ESP_LOGI(TAG, "触控: page=%d widget=0x%04X count=%d",
             data->page_id, data->widget_id, data->event);

    /* ---- 导航按钮: 屏幕内部切页, MCU 延时 100ms 后查询新页面 ---- */
    if (is_nav_widget(data->page_id, data->widget_id)) {
        ESP_LOGI(TAG, "导航按钮: 100ms 后查询新页面");
        /* 重启定时器 (连续快速切页时自动以最后一次为准) */
        esp_timer_stop(s_query_timer);
        esp_timer_start_once(s_query_timer, PAGE_QUERY_DELAY_US);
        return;   /* 导航按钮不进业务分发 */
    }

    /* ---- 普通业务按钮 ---- */
    switch (data->widget_id) {
    case WID_BTN_START:
        ESP_LOGI(TAG, "按钮: 启动");
        cabin_fsm_send_event(FSM_EVT_START);
        break;

    case WID_BTN_PAUSE:
        ESP_LOGI(TAG, "按钮: 暂停");
        cabin_fsm_send_event(FSM_EVT_PAUSE);
        break;

    case WID_BTN_STOP:
        /* 屏幕"停止"按钮在停机时同步取消暂停态: 先发 RESUME 兜底, 再发 RESET。
         * RESUME 在非 PAUSED 状态下被 FSM 忽略, 不会有副作用 */
        ESP_LOGI(TAG, "按钮: 停止");
        cabin_fsm_send_event(FSM_EVT_RESUME);
        cabin_fsm_send_event(FSM_EVT_RESET);
        break;

    case WID_BTN_ESTOP:
        /* ESTOP 在 cabin_fsm_send_event 内部走队列头部, 比其他事件优先处理 */
        ESP_LOGW(TAG, "按钮: 紧急停止!");
        cabin_fsm_send_event(FSM_EVT_ESTOP);
        break;

    case WID_BTN_DISINFECT:
        ESP_LOGI(TAG, "按钮: 消毒");
        cabin_fsm_send_event(FSM_EVT_DISINFECT);
        break;

    case WID_BTN_LIGHT:
        /* 灯光: 切换 (toggle); 不受 FSM 约束, 任何阶段都可手动控制
         * ⚠️ WID_BTN_LIGHT 仍是占位 ID, 真实控件 ID 抓包确认后才会被触发 */
        {
            bool now = actuator_get(CH_LIGHT);
            ESP_LOGI(TAG, "按钮: 灯光 %s → %s", now ? "ON" : "OFF", now ? "OFF" : "ON");
            actuator_set(CH_LIGHT, !now);
        }
        break;

    case WID_BTN_AC:
        /* 空调: 切换; 同上, 占位 ID */
        {
            bool now = actuator_get(CH_AC);
            ESP_LOGI(TAG, "按钮: 空调 %s → %s", now ? "ON" : "OFF", now ? "OFF" : "ON");
            actuator_set(CH_AC, !now);
        }
        break;

    case WID_BTN_SAVE:
        ESP_LOGI(TAG, "按钮: 保存参数");
        // TODO: 读取参数页编辑缓冲区, 调用 cabin_fsm_set_params()
        break;

    case WID_BTN_DEFAULT:
        ESP_LOGI(TAG, "按钮: 恢复默认");
        // TODO: 把参数页编辑缓冲区重置为默认值, 等待用户点保存
        break;

    default:
        ESP_LOGD(TAG, "未处理的控件: 0x%04X", data->widget_id);
        break;
    }
}

/* ============================================================
 * 各页面推送函数 (tx 任务分流)
 *
 * 每页独立, 因为各页控件 ID 空间独立。
 * 所有函数在已持有 485 总线时调用, 不在函数内做 acquire/release。
 *
 * ⚠️ 每个 push_X_page() 函数**必须遵守的约定** ⚠️
 *   1. 如果函数内维护了 static "last_X" 缓存 (delta 推送优化), 必须在函数首段
 *      检查 s_page_force_refresh 标志位, 若为 true 则重置所有缓存并清除标志。
 *   2. 原因: 页面切换后屏幕会把控件恢复到 HMI 默认状态 (显隐/颜色等不被屏幕
 *      缓存), MCU 的 last_X 缓存此时已和屏幕实际状态脱钩, 不重置则永远不会
 *      再推送 → 控件停在默认状态。
 *   3. 模板:
 *        if (s_page_force_refresh) {
 *            s_page_force_refresh = false;
 *            // 重置本页所有 last_X 为"强制下一轮重推"的值:
 *            //   bool   → 设为当前实际状态的相反值
 *            //   int    → 设为 INT_MIN 或越界哨兵值
 *        }
 *   4. 如果函数内**没有**缓存 (每轮都全量推送), 可以不处理这个标志位,
 *      但推荐仍然消费掉它避免影响其他 push_X_page。
 * ============================================================ */

/**
 * @brief 第 0 页 (主监控) 推送逻辑
 *
 * WiFi/MQTT 图标 + 时间 + 温湿度 (传感器联动)
 * 压力/氧气/FSM 状态/进度条等在对应服务实装后启用
 */
static void push_main_page(void)
{
    char buf[32];

    /* 记录上一次的连接状态，仅在变化时才发送指令，避免重复刷屏 */
    static bool last_wifi_connected = false;
    static bool last_mqtt_connected = false;

    /* 记录上一次推送的温湿度值 (以 0.1 为单位), 仅变化时刷新 */
    static int last_temp_tenths = INT_MIN;
    static int last_humi_tenths = INT_MIN;
    static int last_press_tenths = INT_MIN;     /* 压力 KPa */
    static int last_o2_tenths    = INT_MIN;     /* 氧浓度 % */

    /* FSM 状态文本: 用指针比较 (中文表是 static, 指针稳定) */
    static const char *last_state_ptr = NULL;

    /* 压力进度条 (0~100): 255 = 强制首次推送 */
    static uint8_t last_progress_pct = 255;

    /* 剩余时间缓存 (分钟): -1 = 强制首次推送 */
    static int last_remain_min = -1;

    /* 时间缓存: -1=从未推送, -2=已显示占位符 "--:--", 0~59=上次推送的分钟 */
    static int last_pushed_min = -1;

    /* 页面刚切回主页: 屏幕端显隐状态被重置为 HMI 默认值, 必须重置本地缓存
     * 强制下面每段都重推, 否则 WiFi/MQTT 图标会停在默认(显示)状态 */
    if (s_page_force_refresh) {
        s_page_force_refresh = false;
        last_wifi_connected = !((xEventGroupGetBits(g_system_events) & EVT_WIFI_CONNECTED) != 0);
        last_mqtt_connected = !((xEventGroupGetBits(g_system_events) & EVT_MQTT_CONNECTED) != 0);
        last_temp_tenths  = INT_MIN;
        last_humi_tenths  = INT_MIN;
        last_press_tenths = INT_MIN;
        last_o2_tenths    = INT_MIN;
        last_state_ptr    = NULL;
        last_progress_pct = 255;
        last_remain_min   = -1;
        last_pushed_min   = -1;
        ESP_LOGI(TAG, "主页强制全量刷新");
    }

    /* --- 推送 WiFi / MQTT 连接状态 --- */
    if (g_system_events) {
        EventBits_t bits = xEventGroupGetBits(g_system_events);
        bool wifi_on = (bits & EVT_WIFI_CONNECTED) != 0;
        bool mqtt_on = (bits & EVT_MQTT_CONNECTED) != 0;

        if (wifi_on != last_wifi_connected) {
            last_wifi_connected = wifi_on;
            screen_set_hidden(WID_WIFI_ICON, wifi_on);
            ESP_LOGI(TAG, "WiFi: %s", wifi_on ? "已连接(隐藏图标)" : "已断开(显示图标)");
        }
        if (mqtt_on != last_mqtt_connected) {
            last_mqtt_connected = mqtt_on;
            screen_set_hidden(WID_MQTT_ICON, mqtt_on);
            ESP_LOGI(TAG, "MQTT: %s", mqtt_on ? "已连接(隐藏图标)" : "已断开(显示图标)");
        }
    }

    /* --- 推送时间 (HH:MM), 每分钟更新一次 --- */
    {
        if (timer_service_is_time_synced()) {
            time_t now;
            struct tm ti;
            time(&now);
            localtime_r(&now, &ti);
            if (ti.tm_min != last_pushed_min) {
                char tbuf[8];
                snprintf(tbuf, sizeof(tbuf), "%02d:%02d", ti.tm_hour, ti.tm_min);
                screen_set_text(WID_TIME, tbuf);
                last_pushed_min = ti.tm_min;
            }
        } else if (last_pushed_min != -2) {
            screen_set_text(WID_TIME, "--:--");
            last_pushed_min = -2;
        }
    }

    /* --- 推送传感器数据 (温湿度/压力/氧浓度, 均以 0.1 为最小分辨率的 delta 判断) --- */
    sensor_data_t sdata;
    if (sensor_service_get_data(&sdata) == ESP_OK) {
        int t_tenths = (int)((sdata.temperature >= 0 ? sdata.temperature + 0.05f
                                                     : sdata.temperature - 0.05f) * 10);
        int h_tenths = (int)(sdata.humidity * 10 + 0.5f);
        int p_tenths = (int)(sdata.pressure_kpa * 10 + 0.5f);
        int o_tenths = (int)(sdata.oxygen_percent * 10 + 0.5f);

        if (t_tenths != last_temp_tenths) {
            snprintf(buf, sizeof(buf), "%.1f", sdata.temperature);
            screen_set_text(WID_TEMP_VALUE, buf);
            last_temp_tenths = t_tenths;
        }
        if (h_tenths != last_humi_tenths) {
            snprintf(buf, sizeof(buf), "%.1f", sdata.humidity);
            screen_set_text(WID_HUMI_VALUE, buf);
            last_humi_tenths = h_tenths;
        }
        if (p_tenths != last_press_tenths) {
            snprintf(buf, sizeof(buf), "%.1f", sdata.pressure_kpa);
            screen_set_text(WID_PRESS_VALUE, buf);
            last_press_tenths = p_tenths;
        }
        if (o_tenths != last_o2_tenths) {
            snprintf(buf, sizeof(buf), "%.1f", sdata.oxygen_percent);
            screen_set_text(WID_O2_VALUE, buf);
            last_o2_tenths = o_tenths;
        }
    }

    /* --- 推送 FSM 状态中文文本 --- */
    {
        const char *state_name = cabin_fsm_get_state_name_cn();
        if (state_name != last_state_ptr) {
            screen_set_text(WID_FSM_STATE, state_name);
            last_state_ptr = state_name;
        }
    }

    /* --- 推送压力进度条 + 联动百分比数字 (两个控件必须同步更新) --- */
    {
        uint8_t pct = cabin_fsm_get_progress_pct();
        if (pct != last_progress_pct) {
            screen_set_val(WID_PRESS_BAR, pct);
            snprintf(buf, sizeof(buf), "%u", pct);
            screen_set_text(WID_PRESS_BAR_PCT, buf);
            last_progress_pct = pct;
        }
    }

    /* --- 推送剩余时间 HH:MM (与顶栏时间同格式, 每分钟刷新) --- */
    {
        cabin_run_info_t rinfo;
        if (cabin_fsm_get_run_info(&rinfo) == ESP_OK) {
            int remain = (int)rinfo.remaining_min;
            if (remain != last_remain_min) {
                snprintf(buf, sizeof(buf), "%02d:%02d", remain / 60, remain % 60);
                screen_set_text(WID_REMAIN_TIME, buf);
                last_remain_min = remain;
            }
        }
    }
}

/**
 * @brief 第 1 页 (参数设置) 推送逻辑
 * TODO: 等参数页控件 ID 抓包确认后实装
 *   显示: 当前目标压力/保压时间/进气速度
 *   涉及控件 ID 待填
 */
static void push_setting_page(void)
{
    /* TODO: 实装时记得遵守顶部约定 —
     * if (s_page_force_refresh) { s_page_force_refresh = false; 重置缓存... } */
}

/**
 * @brief 第 2 页 (执行器状态) 推送逻辑
 * TODO: 等执行器页控件 ID 抓包确认后实装
 * 需要推送 15 路继电器的绿/红指示灯显隐 + ON/OFF 文本
 */
static void push_actuator_page(void)
{
    /* TODO: 读取 actuator bitmap, 逐通道 screen_set_hidden() 控制绿/红灯显隐。
     * 本页控件多 (30+ 个显隐控件), delta 推送优化的收益很大, 务必维护 last_X
     * 缓存并正确处理 s_page_force_refresh 标志位 (参见顶部约定) */
}

/**
 * @brief 第 3 页 (报警记录) 推送逻辑
 * TODO: 等报警页控件 ID 抓包确认后实装
 */
static void push_alarm_page(void)
{
    /* TODO: 从 rule_engine 拉取最近的报警条目, 推送文本。
     * 实装时注意消费 s_page_force_refresh 标志 (参见顶部约定) */
}

/**
 * @brief 第 4 页 (网络设置) 推送逻辑
 * TODO: 等网络页控件 ID 抓包确认后实装
 */
static void push_network_page(void)
{
    /* TODO: WiFi 状态/SSID/IP/信号强度 + MQTT 状态/服务器/上报周期。
     * 涉及多个 set_hidden (绿/红灯指示), 必然受"页面切换缓存失效"影响,
     * 务必遵守 s_page_force_refresh 约定 (参见顶部说明) */
}

/**
 * @brief 第 5 页 (系统信息) 推送逻辑
 *
 * 固件版本和 MCU 型号是静态字段, 只在切入本页时推送一次即可。
 * 因为内容永远不变, 可以用 s_page_force_refresh 作为"首次推送"信号:
 *   - 每次进入本页时标志为 true → 推送一次 → 清除标志
 *   - 后续周期内标志为 false → 跳过, 不浪费总线
 */
static void push_system_page(void)
{
    /* TODO: 固件版本/MCU 型号等静态字段。实装模板:
     *   if (s_page_force_refresh) {
     *       s_page_force_refresh = false;
     *       screen_set_text(WID_FW_VERSION, APP_CFG_FIRMWARE_VERSION);
     *       screen_set_text(WID_MCU_MODEL, "ESP32-S3-WROOM-1-N16R8");
     *   }
     */
}

/* ============================================================
 * 数据推送任务
 *
 * 行为:
 *   - 每 500ms 自然周期刷新 OR 被 xTaskNotifyGive 唤醒立即刷新
 *   - 先处理待发的页面查询 (导航按钮后由定时器触发)
 *   - 然后按当前页面分流执行对应推送函数
 * ============================================================ */

static void screen_tx_task(void *arg)
{
    while (1) {
        /* 等待 500ms 或被唤醒 (切页/查询结果返回) */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SCREEN_REFRESH_MS));

        /* 获取 485 总线 */
        if (bus_acquire(BUS_USER_SCREEN, SCREEN_BUS_TIMEOUT_MS) != ESP_OK) {
            ESP_LOGW(TAG, "获取总线超时，跳过本次刷新");
            continue;
        }

        /* --- 1. 待发的页面查询 (导航按钮后由定时器触发) --- */
        if (s_query_pending) {
            s_query_pending = false;
            esp_err_t ret = screen_query_page();
            ESP_LOGI(TAG, "发送页面查询 %s", ret == ESP_OK ? "成功" : "失败");
            bus_release(BUS_USER_SCREEN);
            /* 不在本轮刷新, 等屏幕响应回来触发 on_page_update → 再次唤醒 tx 任务 */
            continue;
        }

        /* --- 2. 按当前页面分流 --- */
        switch (s_current_page) {
        case PAGE_MAIN:     push_main_page();     break;
        case PAGE_SETTING:  push_setting_page();  break;
        case PAGE_ACTUATOR: push_actuator_page(); break;
        case PAGE_ALARM:    push_alarm_page();    break;
        case PAGE_NETWORK:  push_network_page();  break;
        case PAGE_SYSTEM:   push_system_page();   break;
        default:
            ESP_LOGW(TAG, "未知页面: %d", s_current_page);
            break;
        }

        bus_release(BUS_USER_SCREEN);
    }
}

/* ============================================================
 * 接收解析任务
 * ============================================================ */

static void screen_rx_task(void *arg)
{
    uint8_t rx_buf[64];

    while (1) {
        /* 阻塞式等待总线 (最长 500ms), 期间其他任务持有总线时本任务睡眠,
         * 不会空转也不会产生大量日志。UART 驱动层有 RX ring buffer 吸收
         * 等待期间屏幕发来的字节, 不会丢数据。*/
        if (bus_acquire(BUS_USER_SCREEN, 500) == ESP_OK) {
            int len = uart_read_bytes(APP_CFG_RS485_UART_NUM, rx_buf, sizeof(rx_buf),
                                      pdMS_TO_TICKS(SCREEN_RX_TIMEOUT_MS));
            bus_release(BUS_USER_SCREEN);
            if (len > 0) {
                screen_feed_rx(rx_buf, (uint16_t)len);
            }
        }
        /* 500ms 仍拿不到 — sensor_task 异常长时间占用, 下次循环继续等 */
    }
}

/* ============================================================
 * 服务启动
 * ============================================================ */

esp_err_t screen_service_start(void)
{
    /* 初始化协议层 */
    esp_err_t ret = screen_protocol_init();
    if (ret != ESP_OK) {
        return ret;
    }

    /* 注册触控 / 页面查询响应回调 */
    screen_register_touch_callback(on_touch_event);
    screen_register_page_callback(on_page_update);

    /* 创建页面查询定时器 (one-shot, 导航按钮按下后 100ms 触发) */
    const esp_timer_create_args_t timer_args = {
        .callback = query_timer_cb,
        .name     = "screen_query",
    };
    ret = esp_timer_create(&timer_args, &s_query_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "创建页面查询定时器失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 创建推送任务 — 保存句柄以便其他上下文 xTaskNotifyGive */
    BaseType_t ok = xTaskCreate(screen_tx_task, "screen_tx", SCREEN_TX_TASK_STACK,
                                NULL, SCREEN_TX_TASK_PRIO, &s_tx_task_handle);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "screen_tx_task 创建失败");
        return ESP_FAIL;
    }

    /* 创建接收任务 */
    ok = xTaskCreate(screen_rx_task, "screen_rx", SCREEN_RX_TASK_STACK,
                     NULL, SCREEN_RX_TASK_PRIO, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "screen_rx_task 创建失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "串口屏服务已启动 (刷新周期: %dms, 初始页面: %d)",
             SCREEN_REFRESH_MS, s_current_page);
    return ESP_OK;
}
