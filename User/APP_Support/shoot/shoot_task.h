/**
  * @file       shoot_task.h
  * @brief      发射控制参数与状态结构
  * @note       定义发射模式、摩擦轮参数、拨弹参数和发射状态结构体。
  */
#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "adrc.h"
#include "bsp_fdcan.h"
#include "robot_param.h"
#include "remote_control.h"

/* 射击任务基础配置 */
#ifndef SHOOT_TASK_INIT_TIME
#define SHOOT_TASK_INIT_TIME 200U         // 任务启动延时，单位 ms
#endif

#ifndef SHOOT_CONTROL_TIME
#define SHOOT_CONTROL_TIME 1U             // 射击控制周期，单位 ms
#endif

#ifndef SHOOT_RC_MODE_CHANNEL
#define SHOOT_RC_MODE_CHANNEL 1           // 遥控器射击模式通道
#endif

/* 拨弹轮控制参数 */
#ifndef SHOOT_STRUM_LONG_PRESS_TORQUE_NM
#define SHOOT_STRUM_LONG_PRESS_TORQUE_NM 0.5f
#endif

#ifndef SHOOT_STRUM_SINGLE_TORQUE_FF_NM
#define SHOOT_STRUM_SINGLE_TORQUE_FF_NM 6.0f
#endif

#ifndef SHOOT_STRUM_SINGLE_FF_TIME_MS
#define SHOOT_STRUM_SINGLE_FF_TIME_MS 100U
#endif

#ifndef SHOOT_STRUM_SINGLE_FF_RELEASE_MS
#define SHOOT_STRUM_SINGLE_FF_RELEASE_MS 80U
#endif

#ifndef SHOOT_STRUM_SINGLE_FF_FILTER_ALPHA
#define SHOOT_STRUM_SINGLE_FF_FILTER_ALPHA 0.9f
#endif

#ifndef SHOOT_STRUM_TORQUE_PID_KP
#define SHOOT_STRUM_TORQUE_PID_KP 7.60f
#endif

#ifndef SHOOT_STRUM_TORQUE_PID_KI
#define SHOOT_STRUM_TORQUE_PID_KI 0.0f
#endif

#ifndef SHOOT_STRUM_TORQUE_PID_KD
#define SHOOT_STRUM_TORQUE_PID_KD 0.20f
#endif

#ifndef SHOOT_STRUM_TORQUE_PID_MAX_OUT
#define SHOOT_STRUM_TORQUE_PID_MAX_OUT 7.5f
#endif

#ifndef SHOOT_STRUM_TORQUE_PID_MAX_IOUT
#define SHOOT_STRUM_TORQUE_PID_MAX_IOUT 7.5f
#endif

#ifndef SHOOT_STRUM_POS_DEADBAND
#define SHOOT_STRUM_POS_DEADBAND 0.02f
#endif

/* 摩擦轮目标与保护配置 */
#ifndef SHOOT_FRIC_TARGET_SPEED_RPM
#define SHOOT_FRIC_TARGET_SPEED_RPM 5130  // 三路摩擦轮统一目标转速，单位 rpm
#endif

#ifndef SHOOT_FRIC_WHEEL_RADIUS_M
#define SHOOT_FRIC_WHEEL_RADIUS_M 0.03f   // 摩擦轮半径，单位 m
#endif

#ifndef SHOOT_FRIC_MAX_CURRENT
#define SHOOT_FRIC_MAX_CURRENT 5       // 三路摩擦轮电流上限，单位 A
#endif

#ifndef SHOOT_FRIC_CURRENT_CMD_FULL_SCALE
#define SHOOT_FRIC_CURRENT_CMD_FULL_SCALE 16384.0f
#endif

#ifndef SHOOT_FRIC_CURRENT_FULL_SCALE_A
#define SHOOT_FRIC_CURRENT_FULL_SCALE_A 20.0f
#endif

#ifndef SHOOT_FRIC_OUTPUT_TORQUE_CONSTANT_NM_PER_A
#define SHOOT_FRIC_OUTPUT_TORQUE_CONSTANT_NM_PER_A 0.3f
#endif

#ifndef SHOOT_FRIC_REDUCTION_RATIO
#define SHOOT_FRIC_REDUCTION_RATIO (3591.0f / 187.0f)
#endif

#ifndef SHOOT_FRIC_FEEDBACK_RANGE_RPM
#define SHOOT_FRIC_FEEDBACK_RANGE_RPM 7000 // 速度反馈量程估计，用于 ADRC 参数设计
#endif

#ifndef SHOOT_FRIC_FDB_TIMEOUT
#define SHOOT_FRIC_FDB_TIMEOUT 100U       // 反馈超时保护，单位 ms
#endif

#ifndef SHOOT_FRIC_TEMP_LIMIT
#define SHOOT_FRIC_TEMP_LIMIT 80U         // 电机温度保护阈值
#endif

/* fric1 ADRC 参数 */
#ifndef SHOOT_FRIC1_B0
#define SHOOT_FRIC1_B0 10000.0f
#endif

#ifndef SHOOT_FRIC1_RESPONSE_TIME_S
#define SHOOT_FRIC1_RESPONSE_TIME_S 0.01269442f
#endif

#ifndef SHOOT_FRIC1_OBSERVER_RATIO
#define SHOOT_FRIC1_OBSERVER_RATIO 2.5f
#endif

#ifndef SHOOT_FRIC1_OUTPUT_RATE_LIMIT
#define SHOOT_FRIC1_OUTPUT_RATE_LIMIT 1000
#endif

/* fric2 ADRC 参数 */
#ifndef SHOOT_FRIC2_B0
#define SHOOT_FRIC2_B0 10000.0f
#endif

#ifndef SHOOT_FRIC2_RESPONSE_TIME_S
#define SHOOT_FRIC2_RESPONSE_TIME_S 0.01269442f
#endif

#ifndef SHOOT_FRIC2_OBSERVER_RATIO
#define SHOOT_FRIC2_OBSERVER_RATIO 2.5f
#endif

#ifndef SHOOT_FRIC2_OUTPUT_RATE_LIMIT
#define SHOOT_FRIC2_OUTPUT_RATE_LIMIT 1000
#endif

/* fric3 ADRC 参数 */
#ifndef SHOOT_FRIC3_B0
#define SHOOT_FRIC3_B0 10000.0f
#endif

#ifndef SHOOT_FRIC3_RESPONSE_TIME_S
#define SHOOT_FRIC3_RESPONSE_TIME_S 0.01269442f
#endif

#ifndef SHOOT_FRIC3_OBSERVER_RATIO
#define SHOOT_FRIC3_OBSERVER_RATIO 2.5f
#endif

#ifndef SHOOT_FRIC3_OUTPUT_RATE_LIMIT
#define SHOOT_FRIC3_OUTPUT_RATE_LIMIT 1000
#endif

/* ADRC 非线性项配置 */
#ifndef SHOOT_FRIC_ERROR_LINEAR_ZONE
#define SHOOT_FRIC_ERROR_LINEAR_ZONE 120  // fal 线性区，增大后误差段更平缓
#endif

#ifndef SHOOT_FRIC_ALPHA1
#define SHOOT_FRIC_ALPHA1 0.5f            // 控制律 fal 指数
#endif

#ifndef SHOOT_FRIC_ALPHA2
#define SHOOT_FRIC_ALPHA2 0.25f           // ESO fal 指数
#endif

/* 掉速触发前馈补偿配置 */
#ifndef SHOOT_FRIC_FF_ENABLE
#define SHOOT_FRIC_FF_ENABLE 1            // 1: 使能掉速触发前馈，0: 关闭
#endif

#ifndef SHOOT_FRIC1_FF_TRIGGER_DROP_RPM
#define SHOOT_FRIC1_FF_TRIGGER_DROP_RPM 300.0f // fric1 掉速触发阈值
#endif

#ifndef SHOOT_FRIC1_FF_MIN_SPEED_RATIO
#define SHOOT_FRIC1_FF_MIN_SPEED_RATIO 0.85f   // fric1 进入稳定区后才允许触发前馈
#endif

#ifndef SHOOT_FRIC1_FF_CURRENT
#define SHOOT_FRIC1_FF_CURRENT 3.2f        // fric1 固定前馈电流，单位 A
#endif

#ifndef SHOOT_FRIC1_FF_DURATION_MS
#define SHOOT_FRIC1_FF_DURATION_MS 50U     // fric1 前馈持续时间，单位 ms
#endif

#ifndef SHOOT_FRIC1_FF_COOLDOWN_MS
#define SHOOT_FRIC1_FF_COOLDOWN_MS 20U     // fric1 前馈冷却时间，单位 ms
#endif

#ifndef SHOOT_FRIC2_FF_TRIGGER_DROP_RPM
#define SHOOT_FRIC2_FF_TRIGGER_DROP_RPM 300.0f // fric2 掉速触发阈值
#endif

#ifndef SHOOT_FRIC2_FF_MIN_SPEED_RATIO
#define SHOOT_FRIC2_FF_MIN_SPEED_RATIO 0.85f   // fric2 进入稳定区后才允许触发前馈
#endif

#ifndef SHOOT_FRIC2_FF_CURRENT
#define SHOOT_FRIC2_FF_CURRENT 3.2f        // fric2 固定前馈电流，单位 A
#endif

#ifndef SHOOT_FRIC2_FF_DURATION_MS
#define SHOOT_FRIC2_FF_DURATION_MS 50U     // fric2 前馈持续时间，单位 ms
#endif

#ifndef SHOOT_FRIC2_FF_COOLDOWN_MS
#define SHOOT_FRIC2_FF_COOLDOWN_MS 20U     // fric2 前馈冷却时间，单位 ms
#endif

#ifndef SHOOT_FRIC3_FF_TRIGGER_DROP_RPM
#define SHOOT_FRIC3_FF_TRIGGER_DROP_RPM 400.0f // fric3 掉速触发阈值
#endif

#ifndef SHOOT_FRIC3_FF_MIN_SPEED_RATIO
#define SHOOT_FRIC3_FF_MIN_SPEED_RATIO 0.85f   // fric3 进入稳定区后才允许触发前馈
#endif

#ifndef SHOOT_FRIC3_FF_CURRENT
#define SHOOT_FRIC3_FF_CURRENT 3.2f        // fric3 固定前馈电流，单位 A
#endif

#ifndef SHOOT_FRIC3_FF_DURATION_MS
#define SHOOT_FRIC3_FF_DURATION_MS 50U     // fric3 前馈持续时间，单位 ms
#endif

#ifndef SHOOT_FRIC3_FF_COOLDOWN_MS
#define SHOOT_FRIC3_FF_COOLDOWN_MS 20U     // fric3 前馈冷却时间，单位 ms
#endif

/* 三路摩擦轮安装方向 */
#ifndef SHOOT_FRIC1_DIRECTION
#define SHOOT_FRIC1_DIRECTION -1          // fric1 实际安装方向
#endif

#ifndef SHOOT_FRIC2_DIRECTION
#define SHOOT_FRIC2_DIRECTION 1           // fric2 实际安装方向
#endif

#ifndef SHOOT_FRIC3_DIRECTION
#define SHOOT_FRIC3_DIRECTION 1           // fric3 实际安装方向
#endif

/* 发射热量模型配置 */
#ifndef SHOOT_HEAT_PER_BULLET
#define SHOOT_HEAT_PER_BULLET 100U        // 单发弹丸增加热量
#endif

#ifndef SHOOT_HEAT_LIMIT
#define SHOOT_HEAT_LIMIT 200U             // 热量上限，预测达到或超过该值时禁止拨弹
#endif

#ifndef SHOOT_HEAT_COOL_PER_SECOND
#define SHOOT_HEAT_COOL_PER_SECOND 20U    // 每秒自然冷却热量
#endif

#ifndef SHOOT_HEAT_DECAY_INTERVAL_MS
#define SHOOT_HEAT_DECAY_INTERVAL_MS 50U  // 热量每 50 ms 下降 1，对应每秒下降 20
#endif

typedef enum
{
    SHOOT_TASK_STOP = 0,
    SHOOT_TASK_READY_FRIC,
} shoot_task_mode_e;

typedef struct
{
    const motor_measure_t *measure;
    adrc_type_def speed_adrc;
    float speed_rpm;
    float speed_mps;
    float speed_set_rpm;
    float last_speed_rpm;
    float prev_speed_rpm;
    float direction;
    uint16_t ff_ticks;
    uint16_t ff_cooldown_ticks;
    float ff_current;
    int16_t give_current;
    int16_t given_current;
    float give_current_a;
    float given_current_a;
    float give_input_torque_nm;
    float given_input_torque_nm;
} shoot_task_motor_t;

typedef struct
{
    shoot_task_mode_e mode;
    shoot_task_mode_e last_mode;
    const RC_ctrl_t *rc;
    bool friction_enable;
    shoot_task_motor_t fric1;
    shoot_task_motor_t fric2;
    shoot_task_motor_t fric3;
    bool bullet_speed_est_active;
    uint16_t bullet_speed_est_ticks;
    float bullet_speed_start_avg_rpm;
    float bullet_speed_min_fric1_rpm;
    float bullet_speed_min_fric2_rpm;
    float bullet_speed_min_fric3_rpm;
    float bullet_speed_min_avg_rpm;
    float estimated_bullet_speed_mps;
    bool fire_detect_active;
    bool fire_detected;
    bool fire_detect_latched;
    uint16_t fire_detect_ticks;
    uint16_t fire_detect_latch_ticks;
    uint32_t fired_bullet_count;
    float fire_detect_speed_drop_rpm;
    float fire_detect_current_a;
    uint16_t heat;
    uint16_t heat_cool_ticks;
    bool heat_limit_active;
} shoot_task_control_t;

extern shoot_task_control_t shoot_task_control;

void shoot_init(void);
void shoot_control_loop(void);
void shoot_task_init(void);
void shoot_task_loop(void);

#endif
