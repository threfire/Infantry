/**
 * @file       comm_app.c
 * @brief      设备侧通信应用入口（FreeRTOS 任务）
 * @details    对接 core 层 channel，新版采用“配置优先”的初始化风格：
 *             - time_sync 通道：提供设备/主机时间映射能力
 *             - camera 通道：相机触发事件上报（含可选 GPIO 轮询）
 *             - gimbal 通道：发布云台状态、接收主机增量命令
 *             并通过 ch_uproto_arbiter 实现统一的发送仲裁。
 */
#include "comm_app.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "usb_device.h"
#include "usb_cdc_port.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"

#include "usart.h"
#include "comm_app_config.h"
#if defined(TFMINI_ENABLE) && (TFMINI_ENABLE == 1)
#include "bsp_tfmini.h"
#endif
#include "../../channel/camera/camera_channel.h"
#include "../../channel/camera/camera_config.h"
#include "../../channel/gimbal/gimbal_channel.h"
#include "../../channel/gimbal/gimbal_config.h"
#include "../../channel/time_sync/time_sync_channel.h"
#include "../../channel/time_sync/time_sync_config.h"
#include "../../core/comm.h"
#include "../../core/platform.h"
#include "../../core/uproto.h"
#include "../shared/protocol_ids.h" /* 引入统一的 MUX 消息类型与通道号 */
#include "bsp_dwt.h"
#include "bsp_tim24.h"
#include "gimbal_task.h"
#include "hwt_imu.h"
#include "auto_aim.h"
#include "remote_control.h"

/* 任务配置与通道参数统一放在 comm_app_config.h 中（见 comm_app.h） */

#ifndef COMM_PROTO_ENABLE
#define COMM_PROTO_ENABLE 1
#endif

/* 复用公共协议 ID：UPROTO_MSG_MUX、CAM_CH_ID、TS_CH_ID、GIMBAL_CH_ID 等
 * 自定义演示流通道的 SID 在 comm_app_config.h 中定义 */

static osThreadId_t g_comm_app_tid = NULL;
extern uproto_context_t proto_ctx;

static channel_manager_t g_mgr;
static ch_uproto_bind_t g_bind;
static camera_channel_t g_camera;
static gimbal_channel_t g_gimbal;
static time_sync_channel_t g_tsync;

#if defined(TFMINI_ENABLE) && (TFMINI_ENABLE == 1)
static uint32_t g_tfmini_seq = 0;
static uint64_t g_tfmini_last_pub_us = 0;
#endif

typedef struct {
    int32_t robot_id;
    int32_t game_stage;
    int32_t enemy_team;
    int32_t fire_allowed;
    uint16_t status;
} comm_referee_snapshot_t;

__weak uint8_t comm_referee_get_snapshot(comm_referee_snapshot_t *out, uint32_t stale_timeout_ms)
{
    (void)stale_timeout_ms;
    if(!out)
        return 0u;
    out->robot_id = 0;
    out->game_stage = 0;
    out->enemy_team = 0;
    out->fire_allowed = 0;
    out->status = GIMBAL_REFEREE_STATUS_NOT_READY | GIMBAL_REFEREE_STATUS_TIMEOUT;
    return 0u;
}

#ifndef COMM_HOST_CMD_TIMEOUT_MS
#define COMM_HOST_CMD_TIMEOUT_MS 120u
#endif

#ifndef COMM_REFEREE_PUB_PERIOD_MS
#define COMM_REFEREE_PUB_PERIOD_MS 50u
#endif

#ifndef COMM_REFEREE_REQUIRE_QUERY
#define COMM_REFEREE_REQUIRE_QUERY 1
#endif

#ifndef COMM_REFEREE_STALE_TIMEOUT_MS
#define COMM_REFEREE_STALE_TIMEOUT_MS 1000u
#endif

#ifndef COMM_FIRE_SINGLE_PULSE_MS
#define COMM_FIRE_SINGLE_PULSE_MS 20u
#endif

typedef struct {
    uint8_t valid;
    gimbal_fire_cmd_t cmd;
    uint64_t rx_us;
    uint64_t single_pulse_until_us;
    uint64_t last_single_token;
} host_fire_state_t;

typedef struct {
    uint8_t valid;
    gimbal_chassis_cmd_t cmd;
    uint64_t rx_us;
} host_chassis_state_t;

static host_fire_state_t g_host_fire = {0};
static host_chassis_state_t g_host_chassis = {0};
static uint8_t g_rc_injected = 0;
static uint64_t g_referee_last_pub_us = 0;
static uint8_t g_referee_query_seen = 0;
static uint8_t g_referee_force_pub = 0;

/**
 * @brief          备用时间源（微秒）
 * @details        当 DWT 或高精度计时不可用时，用 HAL_GetTick 近似换算
 * @retval         当前时间戳（微秒）
 */
static uint64_t now_us_fallback(void) {
    return ((uint64_t)HAL_GetTick()) * 1000ULL;
}

/**
 * @brief          从时间同步通道获取当前设备时间（微秒）
 * @param[in]      user：保留（未使用）
 * @retval         当前设备时间戳（微秒）
 */
static uint64_t sync_now_us(void *user) {
    (void)user;
    uint64_t now_us = time_sync_channel_now_us(&g_tsync);
    return (now_us != 0ULL) ? now_us : now_us_fallback();
}

/* forward decls for gimbal channel hooks */
static bool gimbal_get_state(gimbal_state_t *out, void *user);
static void gimbal_on_delta(const gimbal_delta_t *d, void *user);
static void gimbal_on_fire(const gimbal_fire_cmd_t *cmd, void *user);
static void gimbal_on_chassis(const gimbal_chassis_cmd_t *cmd, void *user);
static void gimbal_on_referee_query(void *user);
static void comm_apply_host_controls(uint64_t now_us);
static void comm_publish_referee(uint64_t now_us);
#if defined(TFMINI_ENABLE) && (TFMINI_ENABLE == 1)
static void tfmini_tick(void);
#endif

static const float kMdegToRad = 1.7453292519943296e-5f;

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

static RC_ctrl_t *comm_get_rc_ptr(void)
{
    return (RC_ctrl_t *)get_remote_control_point();
}

static uint8_t comm_autoaim_enabled(void)
{
    return (aim.auto_aim_flag == AIM_ON) ? 1u : 0u;
}

static void comm_clear_rc_injection(void)
{
    RC_ctrl_t *rc = comm_get_rc_ptr();
    if(!rc)
        return;

    rc->rc.ch[CHASSIS_X_CHANNEL] = 0;
    rc->rc.ch[CHASSIS_Y_CHANNEL] = 0;
    rc->rc.ch[CHASSIS_WZ_CHANNEL] = 0;
    rc->rc.s[CHASSIS_MODE_CHANNEL] = (char)RC_SW_MID;
    rc->rc.s[CHASSIS_FOLLOW_CHANNEL] = (char)RC_SW_MID;

    rc->mouse.press_l = 0u;
    rc->key.v &= (uint16_t) ~(SHOOT_ON_KEYBOARD | SHOOT_OFF_KEYBOARD);
}

/**
 * @brief          自定义流通道的接收回调
 * @details        将收到的 payload 以相同 SID 回环到主机，用于串口/带宽测试
 * @param[in]      ch：通道对象
 * @param[in]      payload：数据指针
 * @param[in]      len：数据长度
 * @param[in]      user：用户上下文（未使用）
 */
static void stream_on_rx(channel_t *ch, const uint8_t *payload, uint32_t len, void *user) {
    (void)ch;
    (void)user;
    if(!payload || len == 0)
        return;
    (void)ch_uproto_send_notify(&g_bind, COMM_APP_STREAM_CH_ID, COMM_APP_STREAM_SID_DATA, 0, payload, (uint16_t)len);
}

/**
 * @brief          触发一次相机事件
 * @details        供外部 EXTI 中断或其他模块调用，通知 camera 通道上报一次事件
 */
void comm_camera_trigger_pulse(void) {
    camera_channel_trigger(&g_camera);
}

/**
 * @brief          初始化并注册所有通道
 * @details        使用“配置优先”的 EX 初始化接口，显式传入通道参数与时间源
 */
static void setup_channels(void) {
    chmgr_init(&g_mgr);

    /* time sync channel (config-first) */
    ts_time_ops_t ts_ops = {.now_us = (uint64_t (*)(void *))tim24_timebase_now_us, .user = NULL};
    ts_channel_cfg_t ts_cfg = {.ch_id = TS_CH_ID, .period_ms = TS_PERIOD_MS, .initiator = TS_INITIATOR, .priority = TS_PRIORITY, .max_rtt_us = TS_MAX_RTT_US};
    time_sync_channel_init_ex(&g_tsync, &g_bind, &g_mgr, &ts_cfg, &ts_ops);

    /* camera channel (config-first) */
    camera_channel_cfg_t cam_cfg = {.ch_id = CAM_CH_ID, .priority = CAM_PRIORITY};
    camera_channel_init_ex(&g_camera, &g_bind, &g_mgr, &cam_cfg, sync_now_us, NULL, NULL);

    /* gimbal channel (config-first) */
    gimbal_source_ops_t gsrc = {.get_state = gimbal_get_state, .user = NULL, .period_ms = GIMBAL_PUB_PERIOD_MS};
    gimbal_hooks_t ghk = {
      .on_delta = gimbal_on_delta,
      .on_fire = gimbal_on_fire,
      .on_chassis = gimbal_on_chassis,
      .on_referee_query = gimbal_on_referee_query,
      .user = NULL};
    gimbal_channel_cfg_t gcfg = {.ch_id = GIMBAL_CH_ID, .priority = GIMBAL_PRIORITY, .period_ms = 0u};
    gimbal_channel_init_ex(&g_gimbal, &g_bind, &g_mgr, &gcfg, &gsrc, &ghk, sync_now_us, NULL);

    /* stream channel: id=2, default priority=1 */
    static channel_t stream_ch;
    channel_cfg_t st_cfg = {.mode = CH_MODE_QA, .reliable = 0, .expect_reply = 0, .priority = 1, .period_ms = 0};
    channel_hooks_t st_hooks = {.on_rx = stream_on_rx, .on_tick = NULL, .on_cfg_change = NULL};
    channel_init(&stream_ch, COMM_APP_STREAM_CH_ID, &st_cfg, &st_hooks, NULL);
    channel_bind_transport(&stream_ch, &g_bind, UPROTO_MSG_MUX);
    (void)chmgr_register(&g_mgr, &stream_ch);
}

#if defined(TFMINI_ENABLE) && (TFMINI_ENABLE == 1)
/**
 * @brief          Publish TFmini range data (device -> host)
 * @details        Read UART parsed data and pack into gimbal channel
 */
/**
 * @brief          TFmini测距数据发布（设备 → 主机）
 * @details        读取UART解析结果并打包到gimbal通道
 */
static void tfmini_tick(void) {
    uint64_t now = sync_now_us(NULL);
    if(g_tfmini_last_pub_us != 0 &&
       (now - g_tfmini_last_pub_us) < (uint64_t)TFMINI_PUB_PERIOD_MS * 1000ULL) {
        return;
    }
    tfmini_measurement_t m;
    if(!tfmini_read(&m, &g_tfmini_seq))
        return;
    gimbal_tfmini_t msg;
    msg.distance_cm = m.distance_cm;
    msg.strength = m.strength;
    msg.temp_cdeg = m.temp_cdeg;
    msg.status = m.valid ? GIMBAL_TFMINI_STATUS_VALID : 0u;
    msg.ts_us = m.ts_us ? m.ts_us : now;
    (void)gimbal_channel_publish_tfmini(&g_gimbal, &msg);
    g_tfmini_last_pub_us = now;
}
#endif

#if defined(CAM_TRIGGER_ENABLE) && (CAM_TRIGGER_ENABLE == 1)
/**
 * @brief          可选 GPIO 轮询触发
 * @details        当开启 CAM_TRIGGER_ENABLE 时，在任务循环内轮询 GPIO 边沿并触发事件
 */
void comm_camera_trigger_poll(void) {
    static uint8_t prev = 0xFFu;
    uint8_t state = 0u;
    GPIO_PinState s = HAL_GPIO_ReadPin(CAM_TRIGGER_GPIO_PORT, CAM_TRIGGER_GPIO_PIN);
    state = (s == GPIO_PIN_SET) ? 1u : 0u;
    if(prev == 0xFFu) {
        prev = state;
        return;
    }
#if (CAM_TRIGGER_ACTIVE_HIGH == 1)
    if(prev == 0u && state == 1u) {
        comm_camera_trigger_pulse();
    }
#else
    if(prev == 1u && state == 0u) {
        comm_camera_trigger_pulse();
    }
#endif
    prev = state;
}
#endif

/**
 * @brief          通信应用任务入口（FreeRTOS）
 * @param[in]      arg：任务参数（未使用）
 * @details        等待 USB CDC 枚举 → 绑定 uproto 与通道 → 周期性调度
 */
void comm_app_task(void *arg) {
    (void)arg;
    extern USBD_HandleTypeDef hUsbDeviceHS;
    MX_USB_DEVICE_Init();
    usb_cdc_port_init();
#if defined(TFMINI_ENABLE) && (TFMINI_ENABLE == 1)
    /* Init TFmini RX early; do not wait for USB enumeration. */
    tfmini_uart_init(&huart10);
#endif

    uint32_t t0 = HAL_GetTick();
    while((hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) && ((HAL_GetTick() - t0) < COMM_APP_USB_ENUM_TIMEOUT_MS)) {
        osDelay(1);
    }

    ch_uproto_bind(&g_bind, &proto_ctx, UPROTO_MSG_MUX, &g_mgr);
    usb_cdc_port_bind_uproto(&proto_ctx);
    ch_uproto_register_rx(&g_bind);
    setup_channels();

    for(;;) {
#if COMM_PROTO_ENABLE
        usb_cdc_port_poll_rx();
        uproto_tick(&proto_ctx);
        camera_channel_tick(&g_camera);
        ch_uproto_arbiter_tick(&g_bind);
        chmgr_tick(&g_mgr);
        {
            uint64_t now_us = sync_now_us(NULL);
            comm_apply_host_controls(now_us);
            comm_publish_referee(now_us);
        }
#if defined(TFMINI_ENABLE) && (TFMINI_ENABLE == 1)
        tfmini_tick();
#endif
#endif
#if defined(CAM_TRIGGER_ENABLE) && (CAM_TRIGGER_ENABLE == 1)
        comm_camera_trigger_poll();
#endif
        osDelay(COMM_APP_LOOP_DELAY_MS);
    }
}

/**
 * @brief          启动通信应用任务
 * @details        创建并启动 FreeRTOS 任务（防止重复创建）
 */
void comm_app_start(void) {
    if(g_comm_app_tid != NULL)
        return;
    static const osThreadAttr_t comm_app_attributes = {
        .name = "COMM_APP",
        .stack_size = COMM_APP_STACK * 4,
        .priority = (osPriority_t) COMM_APP_PRIO,
    };
    g_comm_app_tid = osThreadNew(comm_app_task, NULL, &comm_app_attributes);
}

/* gimbal integration: real implementations */
/**
 * @brief          云台状态获取回调（设备 → 主机）
 * @param[out]     out：输出状态
 * @param[in]      user：用户上下文（未使用）
 * @retval         true：成功获取；false：不可用
 * @note           示例使用已有的 gimbal/INS 数据源进行单位换算到“微度”
 */
static bool gimbal_get_state(gimbal_state_t *out, void *user) {
    (void)user;
    if(!out)
        return false;
    const gimbal_motor_t *yaw = get_yaw_motor_point();
    const gimbal_motor_t *pit = get_pitch_motor_point();
    const float *imu = get_INS_angle_point();
    const float INV_PI = 0.31830988618379067154f; /* 1/pi */
    if(yaw && pit) {
        out->enc_yaw = (int32_t)(yaw->relative_angle * 180000000.0f * INV_PI);
        out->enc_pitch = (int32_t)(pit->relative_angle * 180000000.0f * INV_PI);
    } else {
        out->enc_yaw = 0;
        out->enc_pitch = 0;
    }
    if(imu) {
        out->yaw_udeg = (int32_t)(imu[0] * 180000000.0f * INV_PI);
        out->pitch_udeg = (int32_t)(imu[1] * 180000000.0f * INV_PI);
        out->roll_udeg = (int32_t)(imu[2] * 180000000.0f * INV_PI);
    } else {
        out->yaw_udeg = out->pitch_udeg = out->roll_udeg = 0;
    }
    out->ts_us = sync_now_us(NULL);
    return true;
}

/**
 * @brief          云台增量命令回调（主机 → 设备）
 * @param[in]      d：增量命令
 * @param[in]      user：用户上下文（未使用）
 * @details        将最新命令写入邮箱，供云台控制任务消费
 */
static void gimbal_on_delta(const gimbal_delta_t *d, void *user) {
    (void)user;
    if(!d)
        return;
    gimbal_cmd_t cmd;
    cmd.delta_yaw_udeg = d->delta_yaw_udeg;
    cmd.delta_pitch_udeg = d->delta_pitch_udeg;
    cmd.status = d->status;
    cmd.ts_us = d->ts_us;
    cmd.version = 0;
    gimbal_mailbox_set(&cmd);
}

static void gimbal_on_fire(const gimbal_fire_cmd_t *cmd, void *user)
{
    (void)user;
    if(!cmd)
        return;
    if(!comm_autoaim_enabled())
        return;

    const uint64_t now_us = sync_now_us(NULL);
    const int32_t prev_fire_on = g_host_fire.cmd.fire_on;

    g_host_fire.cmd = *cmd;
    g_host_fire.rx_us = now_us;
    g_host_fire.valid = 1u;

    if(cmd->fire_on) {
        const uint8_t single_mode = (cmd->fire_mode == 1 || (cmd->fire_mode == 0 && cmd->burst_count <= 1)) ? 1u : 0u;
        if(single_mode) {
            uint8_t need_pulse = 0u;
            if(cmd->ts_us != 0) {
                if(cmd->ts_us != g_host_fire.last_single_token) {
                    need_pulse = 1u;
                    g_host_fire.last_single_token = cmd->ts_us;
                }
            } else if(prev_fire_on == 0) {
                need_pulse = 1u;
                g_host_fire.last_single_token = now_us;
            }
            if(need_pulse) {
                g_host_fire.single_pulse_until_us = now_us + (uint64_t)COMM_FIRE_SINGLE_PULSE_MS * 1000ULL;
            }
        }
    }
}

static void gimbal_on_chassis(const gimbal_chassis_cmd_t *cmd, void *user)
{
    (void)user;
    if(!cmd)
        return;
    if(!comm_autoaim_enabled())
        return;

    g_host_chassis.cmd = *cmd;
    g_host_chassis.rx_us = sync_now_us(NULL);
    g_host_chassis.valid = 1u;
}

static void gimbal_on_referee_query(void *user)
{
    (void)user;
    g_referee_query_seen = 1u;
    g_referee_force_pub = 1u;
}

static void comm_apply_host_controls(uint64_t now_us)
{
    const uint64_t timeout_us = (uint64_t)COMM_HOST_CMD_TIMEOUT_MS * 1000ULL;
    uint8_t fire_active = 0u;
    uint8_t chassis_active = 0u;

    if(!comm_autoaim_enabled()) {
        g_host_fire.valid = 0u;
        g_host_chassis.valid = 0u;
        g_host_fire.single_pulse_until_us = 0;
        if(g_rc_injected) {
            comm_clear_rc_injection();
            g_rc_injected = 0u;
        }
        return;
    }

    if(g_host_fire.valid && (now_us - g_host_fire.rx_us) <= timeout_us)
        fire_active = 1u;
    if(g_host_chassis.valid && (now_us - g_host_chassis.rx_us) <= timeout_us)
        chassis_active = 1u;

    if(g_host_fire.valid && !fire_active) {
        g_host_fire.valid = 0u;
        g_host_fire.single_pulse_until_us = 0;
    }
    if(g_host_chassis.valid && !chassis_active) {
        g_host_chassis.valid = 0u;
    }

    if(!fire_active && !chassis_active) {
        if(g_rc_injected) {
            comm_clear_rc_injection();
            g_rc_injected = 0u;
        }
        return;
    }

    RC_ctrl_t *rc = comm_get_rc_ptr();
    if(!rc)
        return;

    if(chassis_active) {
        const float vx_m_s = (float)g_host_chassis.cmd.vx_mm_s * 0.001f;
        const float vy_m_s = (float)g_host_chassis.cmd.vy_mm_s * 0.001f;
        const float wz_rad_s = (float)g_host_chassis.cmd.wz_mdeg_s * kMdegToRad;

        int32_t ch_x = (int32_t)(vx_m_s / CHASSIS_VX_RC_SEN);
        int32_t ch_y = (int32_t)(-vy_m_s / CHASSIS_VY_RC_SEN);
        int32_t ch_wz = (int32_t)(-wz_rad_s / 0.01f);

        ch_x = clamp_i32(ch_x, -660, 660);
        ch_y = clamp_i32(ch_y, -660, 660);
        ch_wz = clamp_i32(ch_wz, -660, 660);

        rc->rc.ch[CHASSIS_X_CHANNEL] = (int16_t)ch_x;
        rc->rc.ch[CHASSIS_Y_CHANNEL] = (int16_t)ch_y;
        rc->rc.ch[CHASSIS_WZ_CHANNEL] = (int16_t)ch_wz;

        switch(g_host_chassis.cmd.mode) {
            default:
            case 0: /* no follow yaw */
                rc->rc.s[CHASSIS_MODE_CHANNEL] = (char)RC_SW_DOWN;
                rc->rc.s[CHASSIS_FOLLOW_CHANNEL] = (char)RC_SW_UP;
                break;
            case 1: /* follow gimbal yaw */
                rc->rc.s[CHASSIS_MODE_CHANNEL] = (char)RC_SW_DOWN;
                rc->rc.s[CHASSIS_FOLLOW_CHANNEL] = (char)RC_SW_DOWN;
                break;
            case 2: /* spin */
                rc->rc.s[CHASSIS_MODE_CHANNEL] = (char)RC_SW_UP;
                rc->rc.s[CHASSIS_FOLLOW_CHANNEL] = (char)RC_SW_MID;
                break;
            case 3: /* no move */
                rc->rc.s[CHASSIS_MODE_CHANNEL] = (char)RC_SW_MID;
                rc->rc.s[CHASSIS_FOLLOW_CHANNEL] = (char)RC_SW_MID;
                break;
        }
    } else {
        rc->rc.ch[CHASSIS_X_CHANNEL] = 0;
        rc->rc.ch[CHASSIS_Y_CHANNEL] = 0;
        rc->rc.ch[CHASSIS_WZ_CHANNEL] = 0;
        rc->rc.s[CHASSIS_MODE_CHANNEL] = (char)RC_SW_MID;
        rc->rc.s[CHASSIS_FOLLOW_CHANNEL] = (char)RC_SW_MID;
    }

    rc->key.v &= (uint16_t) ~(SHOOT_ON_KEYBOARD | SHOOT_OFF_KEYBOARD);
    if(fire_active && g_host_fire.cmd.fire_on) {
        const uint8_t continuous_mode = (g_host_fire.cmd.fire_mode == 2 || g_host_fire.cmd.burst_count > 1) ? 1u : 0u;
        rc->key.v |= SHOOT_ON_KEYBOARD;
        if(continuous_mode) {
            rc->mouse.press_l = 1u;
        } else {
            rc->mouse.press_l = (now_us <= g_host_fire.single_pulse_until_us) ? 1u : 0u;
        }
    } else {
        rc->mouse.press_l = 0u;
        rc->key.v |= SHOOT_OFF_KEYBOARD;
    }

    g_rc_injected = 1u;
}

static void comm_publish_referee(uint64_t now_us)
{
#if COMM_REFEREE_REQUIRE_QUERY
    if(!g_referee_query_seen) {
        return;
    }
#endif

    if(!g_referee_force_pub &&
       g_referee_last_pub_us != 0 &&
       (now_us - g_referee_last_pub_us) < (uint64_t)COMM_REFEREE_PUB_PERIOD_MS * 1000ULL) {
        return;
    }

    gimbal_referee_t msg;
    comm_referee_snapshot_t snap;
    (void)comm_referee_get_snapshot(&snap, COMM_REFEREE_STALE_TIMEOUT_MS);
    msg.robot_id = snap.robot_id;
    msg.game_stage = snap.game_stage;
    msg.enemy_team = snap.enemy_team;
    msg.fire_allowed = snap.fire_allowed;
    uint16_t status = snap.status;

    if(!((snap.robot_id >= 1 && snap.robot_id <= 99) || (snap.robot_id >= 100 && snap.robot_id <= 199))) {
        status |= GIMBAL_REFEREE_STATUS_ERROR;
    }
    msg.status = status;
    msg.ts_us = now_us;

    (void)gimbal_channel_publish_referee(&g_gimbal, &msg);
    g_referee_last_pub_us = now_us;
    g_referee_force_pub = 0u;
}
