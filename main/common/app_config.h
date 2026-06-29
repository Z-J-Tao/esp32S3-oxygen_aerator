#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ============================================================
 * 引脚定义 — 与原理图 Schematic3 V3.0 一一对应
 * ============================================================ */

// UART1 - RS485 (Modbus 传感器 + 尚视界串口屏，经 TTL 转 485 模块)
// 硬件：TTL 转 485 模块（内置自动 DE/RE 方向控制，无需额外方向引脚）
#define APP_CFG_RS485_UART_NUM       UART_NUM_1
#define APP_CFG_RS485_TX_PIN         17      // GPIO17 → TTL转485模块 DI → RS485 总线 A+/B-
#define APP_CFG_RS485_RX_PIN         18      // GPIO18 ← TTL转485模块 RO ← RS485 总线 A+/B-
#define APP_CFG_RS485_BAUD_RATE      9600
// 注：GPIO8 原为 RS485 DE 方向控制引脚，现 TTL转485 模块自动控制方向，GPIO8 空闲预留

// UART2 - ASR PRO 语音模块
#define APP_CFG_VOICE_UART_NUM       UART_NUM_2
#define APP_CFG_VOICE_TX_PIN         11      // GPIO11 → U2TXD → ASR_PRO_RX (PA3)
#define APP_CFG_VOICE_RX_PIN         12      // GPIO12 → U2RXD → ASR_PRO_TX (PA2)
#define APP_CFG_VOICE_BAUD_RATE      9600

// I2C - PCA9685PW (16路 PWM → 继电器/执行器)
#define APP_CFG_I2C_NUM              I2C_NUM_0
#define APP_CFG_I2C_SDA_PIN          1       // GPIO1 → IO_SDA
#define APP_CFG_I2C_SCL_PIN          2       // GPIO2 → IO_SCL
#define APP_CFG_I2C_FREQ_HZ          100000
#define APP_CFG_PCA9685_ADDR         0x40    // A0-A5 全接地，地址 0x40

// 应急按钮
#define APP_CFG_EMERGENCY_BTN_PIN    3       // GPIO3 → 应急按钮（低电平触发）

// 调试 LED
#define APP_CFG_LED_PIN              48      // GPIO48 → LED1 调试灯

// USB 调试串口 (UART0 → CH340C，系统默认，无需手动配置)
// TXD0 → U0TXD → CH340C RXD
// RXD0 → U0RXD → CH340C TXD

/* ============================================================
 * RS485 硬件说明 (无需软件配置，仅作记录)
 * 使用 TTL 转 485 模块，模块内部自动控制收发方向 (DE/RE)
 * ESP32 仅需连接 TX/RX 两线 + VCC/GND 供电
 * 原光耦隔离电路 (HCPL0600×3 + F0505S-1WR3 + SSP3485) 已废弃
 * ============================================================ */

/* ==================== Modbus 配置 ==================== */

#define APP_CFG_MODBUS_MAX_DEVICES   16      // 最大从站设备数
#define APP_CFG_MODBUS_POLL_MS       2000    // 传感器轮询间隔 (ms)，rule_engine/screen 均依赖此频率
#define APP_CFG_MQTT_REPORT_INTERVAL_MS  10000   // MQTT 上报间隔 (ms)，独立于采集频率

/* ==================== 执行器配置 ==================== */

#define APP_CFG_ACTUATOR_MAX_CH      16      // PCA9685 通道数 (PWM0-PWM15)

/* ==================== WiFi 配置（暂时硬编码，串口屏接入后改为 NVS 读取） ==================== */

/* 主 WiFi */
#define APP_CFG_WIFI_SSID            "vivo Y31s"
#define APP_CFG_WIFI_PASSWORD        "88888888"

/* 备用 WiFi — 主WiFi连续失败后自动切换 */
#define APP_CFG_WIFI_SSID_BACKUP     "sundisplay_2.4G"
#define APP_CFG_WIFI_PASSWORD_BACKUP "ssj88888888"

/* 连续断线多少次后切换到另一组 WiFi */
#define APP_CFG_WIFI_SWITCH_THRESHOLD  3

/* ==================== MQTT 配置 ==================== */

/* EMQX Cloud (cn-hangzhou 共享集群, TLS 8883)
 * 数据流: ESP32 → EMQX Cloud → 规则引擎 HTTP → cpolar → ThingsBoard */
#define APP_CFG_MQTT_BROKER_URI      "mqtts://ie1f9c8f.ala.cn-hangzhou.emqxsl.cn:8883"

/* EMQX 认证 */
#define APP_CFG_MQTT_USERNAME        "User001"
#define APP_CFG_MQTT_PASSWORD        "User001"
#define APP_CFG_MQTT_CLIENT_ID       "ESP32S3_001"

/* 设备 ID（用于构造主题前缀 oxy/device/{deviceId}/...） */
#define APP_CFG_MQTT_DEVICE_ID       "ESP32S3_001"

/* MQTT 主题（格式: oxy/device/{deviceId}/{type}） */
#define APP_CFG_MQTT_REPORT_TOPIC    "oxy/device/" APP_CFG_MQTT_DEVICE_ID "/data"
#define APP_CFG_MQTT_STATUS_TOPIC    "oxy/device/" APP_CFG_MQTT_DEVICE_ID "/status"
#define APP_CFG_MQTT_ALERT_TOPIC     "oxy/device/" APP_CFG_MQTT_DEVICE_ID "/alert"
#define APP_CFG_MQTT_CMD_TOPIC       "oxy/device/" APP_CFG_MQTT_DEVICE_ID "/cmd"
#define APP_CFG_MQTT_PARAM_TOPIC     "oxy/device/" APP_CFG_MQTT_DEVICE_ID "/param"

/* 固件版本 */
#define APP_CFG_FIRMWARE_VERSION     "1.0.0"

/* ThingsBoard Access Token（供 EMQX 规则引擎 HTTP 转发时使用，仅作记录） */
#define APP_CFG_TB_ACCESS_TOKEN      "GFdaO9KdILx0P8KOQQ0T"

/* ---- cpolar 双推高可用方案 ----
 * cpolar 域名会在 .top 和 .cn 之间切换，单一 URL 不可靠。
 * 解决方案：在 EMQX Cloud 中创建 **两条规则 + 两个 HTTP 连接器**，
 *           分别指向 .top 和 .cn，同时推送。ThingsBoard 遥测 API 天然幂等，
 *           重复写入同一时间戳的数据不会产生副作用。
 *
 * EMQX 规则 SQL（两条规则使用相同 SQL）:
 *   SELECT * FROM "oxy/device/+/data"
 *
 * HTTP 连接器公共配置:
 *   Method : POST
 *   Header : content-type: application/json
 *   Body   : {"pressure":${payload.pressure},"oxygen":${payload.oxygen},
 *             "temperature":${payload.temperature},"humidity":${payload.humidity}}
 *
 * 连接器 1 (主 - .top):  URL = APP_CFG_TB_HTTP_ENDPOINT
 * 连接器 2 (备 - .cn ):  URL = APP_CFG_TB_HTTP_ENDPOINT_BACKUP
 * -------------------------------- */
#define APP_CFG_TB_HTTP_ENDPOINT         "http://oxysys.cpolar.top/api/v1/" APP_CFG_TB_ACCESS_TOKEN "/telemetry"
#define APP_CFG_TB_HTTP_ENDPOINT_BACKUP  "http://oxysys.cpolar.cn/api/v1/" APP_CFG_TB_ACCESS_TOKEN "/telemetry"

/* ==================== 规则引擎 ==================== */

#define APP_CFG_RULE_MAX_COUNT       32      // 最大规则数

/* ── 报警阈值默认值（可通过 NVS 覆盖）── */

/* 温度（两级，仅报警不停机） */
#define APP_CFG_ALARM_TEMP_WARN       40.0f   // 温度警告 (℃) — 语音 + MQTT
#define APP_CFG_ALARM_TEMP_FAULT      50.0f   // 温度故障 (℃) — 语音 + MQTT + FSM_EVT_FAULT

/* 湿度（高低两端，仅报警不停机） */
#define APP_CFG_ALARM_HUMIDITY_LOW    30.0f   // 低湿警告 (%RH) — 语音 + MQTT
#define APP_CFG_ALARM_HUMIDITY_HIGH   80.0f   // 高湿警告 (%RH) — 语音 + MQTT

/* 氧浓度（三级，高氧方向） */
#define APP_CFG_ALARM_O2_LEVEL1       22.0f   // 一级预警 (%) — 关增氧机+供氧阀，开进/排气阀换气
#define APP_CFG_ALARM_O2_LEVEL2       25.0f   // 二级报警 (%) — 全开进/排气阀加大换气 + 声光报警
#define APP_CFG_ALARM_O2_LEVEL3       30.0f   // 三级故障 (%) — 紧急泄压 + 提示出舱 + 安全模式

/* 压力（三级软件 + 一级机械） */
#define APP_CFG_ALARM_PRESSURE_WARN   58.0f   // 预警 (KPa) — 停止增压
#define APP_CFG_ALARM_PRESSURE_FAULT  60.0f   // 超限 (KPa) — 关空气泵 + 打开排气阀 + 异常保护
#define APP_CFG_ALARM_PRESSURE_EMERG  62.0f   // 极端超压 (KPa) — 排气阀全开 + 快速泄压 + 紧急模式
/* 70 KPa: 机械泄压阀自动开启（硬件保护，不经软件） */

/* 压力变化率 */
#define APP_CFG_ALARM_PRESSURE_RATE   15.0f   // 最大允许升/降压速率 (KPa/min)，超出则停机

/* 传感器离线 */
#define APP_CFG_ALARM_SENSOR_TIMEOUT  3       // 连续超时次数 → 故障停机

/* ==================== 定时任务 ==================== */

#define APP_CFG_TIMER_MAX_TASKS      16      // 最大定时任务数
#define APP_CFG_SNTP_SERVER          "ntp.aliyun.com"     // 阿里云 NTP（国内稳定）
#define APP_CFG_SNTP_SERVER_BACKUP   "ntp.tencent.com"    // 腾讯云 NTP（国内备用）
#define APP_CFG_TIMEZONE             "CST-8"              // 中国标准时间 UTC+8

#endif // APP_CONFIG_H
