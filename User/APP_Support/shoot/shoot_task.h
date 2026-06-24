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
#include "pid.h"
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

#ifndef SHOOT_STRUM_REDUCTION_RATIO
#define SHOOT_STRUM_REDUCTION_RATIO 36.0f  // 2006 拨弹电机减速比，电机侧反馈/输出轴运动 = 36
#endif

#ifndef SHOOT_STRUM_SINGLE_STEP_ECD
#define SHOOT_STRUM_SINGLE_STEP_ECD (1024.0f * SHOOT_STRUM_REDUCTION_RATIO) // 2006 电机侧单步目标，对应拨盘输出轴 1/8 圈
#endif

#ifndef SHOOT_STRUM_CMD_KP
#define SHOOT_STRUM_CMD_KP 0.05f            // 2006 拨弹位置误差到电流命令的比例系数，单位 cmd/ecd
#endif

#ifndef SHOOT_STRUM_CMD_KD
#define SHOOT_STRUM_CMD_KD 0.01f            // 2006 拨弹速度阻尼系数，单位 cmd/rpm
#endif

#ifndef SHOOT_STRUM_CMD_MAX
#define SHOOT_STRUM_CMD_MAX 8192.0f         // 2006 拨弹电流命令限幅
#endif

#ifndef SHOOT_STRUM_POS_DEADBAND_ECD
#define SHOOT_STRUM_POS_DEADBAND_ECD 20.0f  // 2006 拨弹到位死区，单位编码器刻度
#endif

#ifndef SHOOT_STRUM_CONTINUE_SPEED_RPM
#define SHOOT_STRUM_CONTINUE_SPEED_RPM 150.0f // 拨盘输出轴连发目标转速，单位 rpm
#endif

#ifndef SHOOT_STRUM_SPEED_PID_KP
#define SHOOT_STRUM_SPEED_PID_KP 24.0f        // 2006 拨弹连发速度 PID 比例系数，单位 cmd/rpm
#endif

#ifndef SHOOT_STRUM_SPEED_PID_KI
#define SHOOT_STRUM_SPEED_PID_KI 0.0f         // 2006 拨弹连发速度 PID 积分系数
#endif

#ifndef SHOOT_STRUM_SPEED_PID_KD
#define SHOOT_STRUM_SPEED_PID_KD 1.0f         // 2006 拨弹连发速度 PID 微分系数
#endif

#ifndef SHOOT_STRUM_SPEED_PID_MAX_OUT
#define SHOOT_STRUM_SPEED_PID_MAX_OUT 6500.0f // 2006 拨弹连发速度 PID 输出限幅，单位 cmd
#endif

#ifndef SHOOT_STRUM_SPEED_PID_MAX_IOUT
#define SHOOT_STRUM_SPEED_PID_MAX_IOUT 0.0f   // 2006 拨弹连发速度 PID 积分限幅，单位 cmd
#endif

#ifndef SHOOT_STRUM_CONTINUE_FF_CMD
#define SHOOT_STRUM_CONTINUE_FF_CMD 1000.0f   // 2006 拨弹连发克服静摩擦的前馈电流命令
#endif

#ifndef SHOOT_STRUM_CONTINUE_CMD_MAX
#define SHOOT_STRUM_CONTINUE_CMD_MAX 8192.0f  // 2006 拨弹连发电流命令限幅
#endif

#ifndef SHOOT_STRUM_BLOCK_SPEED_RPM
#define SHOOT_STRUM_BLOCK_SPEED_RPM 20.0f     // 2006 拨弹堵转判定转速阈值，单位 rpm
#endif

#ifndef SHOOT_STRUM_BLOCK_CURRENT_CMD
#define SHOOT_STRUM_BLOCK_CURRENT_CMD 4500    // 2006 拨弹堵转判定反馈电流命令阈值
#endif

#ifndef SHOOT_STRUM_BLOCK_TIME_MS
#define SHOOT_STRUM_BLOCK_TIME_MS 120U        // 2006 拨弹低速大电流持续该时间后触发反转，单位 ms
#endif

#ifndef SHOOT_STRUM_REVERSE_TIME_MS
#define SHOOT_STRUM_REVERSE_TIME_MS 140U      // 2006 拨弹堵转反转退弹时间，单位 ms
#endif

#ifndef SHOOT_STRUM_REVERSE_CMD
#define SHOOT_STRUM_REVERSE_CMD 3500.0f       // 2006 拨弹堵转反转电流命令
#endif

#ifndef SHOOT_STRUM_RETREAT_RC_CHANNEL
#define SHOOT_STRUM_RETREAT_RC_CHANNEL 3      // 左摇杆竖直通道，拉到最低触发退弹
#endif

#ifndef SHOOT_STRUM_RETREAT_RC_THRESHOLD
#define SHOOT_STRUM_RETREAT_RC_THRESHOLD (-600) // 左摇杆下拉判定阈值
#endif

#ifndef SHOOT_STRUM_RETREAT_SPEED_RPM
#define SHOOT_STRUM_RETREAT_SPEED_RPM 120.0f  // 拨盘输出轴持续退弹目标转速，单位 rpm
#endif

#ifndef SHOOT_STRUM_RETREAT_FF_CMD
#define SHOOT_STRUM_RETREAT_FF_CMD 2000.0f    // 2006 持续退弹克服阻力的前馈电流命令
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
    bool target_valid;              // 拨弹目标是否已经初始化
    bool last_switch_up;            // 上一周期拨弹拨杆是否为 UP
    bool last_switch_down;          // 上一周期拨弹拨杆是否为 DOWN
    bool last_switch_mid;           // 上一周期拨弹拨杆是否为 MID
    bool switch_up;                 // 当前周期拨弹拨杆是否为 UP
    bool switch_down;               // 当前周期拨弹拨杆是否为 DOWN
    bool switch_mid;                // 当前周期拨弹拨杆是否为 MID
    bool strum_ready;               // 拨弹控制允许状态
    bool feedback_ready;            // 拨弹 2006 反馈有效状态
    float target_ecd;               // 拨弹目标连续编码器刻度，单位为 2006 电机侧编码器刻度
    float target_cmd_ecd;           // 拨弹低通后的目标连续编码器刻度
    float feedback_ecd;             // 拨弹当前单圈编码器刻度
    float feedback_ecd_last;        // 拨弹上一周期单圈编码器刻度
    float feedback_ecd_continuous;  // 拨弹连续编码器刻度
    int32_t feedback_round;         // 拨弹编码器跨 8192/0 后的圈数
    float position_error_ecd;       // 拨弹位置误差，单位编码器刻度
    uint16_t switch_hold_ticks;      // 拨弹拨杆保持时间，单位控制周期
    uint16_t block_ticks;            // 拨弹堵转持续计数，单位控制周期
    uint16_t reverse_ticks;          // 拨弹反转退弹剩余计数，单位控制周期
    bool continuous_active;          // 拨弹连发速度控制状态
    bool single_active;              // 拨弹单发位置控制状态
    bool single_pending;             // 拨弹单发候选状态
    bool retreat_active;             // 拨弹持续退弹状态
    bool speed_pid_active;           // 拨弹速度 PID 运行状态
    pid_type_def speed_pid;          // 拨弹连发/退弹速度 PID
    int16_t current_cmd;            // 拨弹发送到 2006 电调的电流命令
} shoot_task_strum_t;

typedef struct
{
    shoot_task_mode_e mode;
    shoot_task_mode_e last_mode;
    const RC_ctrl_t *rc;
    bool friction_enable;
    shoot_task_motor_t fric1;
    shoot_task_motor_t fric2;
    shoot_task_motor_t fric3;
    shoot_task_strum_t strum;
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
