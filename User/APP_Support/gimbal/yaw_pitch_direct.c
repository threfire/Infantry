/**
  * @file       yaw_pitch_direct.c
  * @brief      yaw/pitch 直控云台控制实现
  * @note       实现云台反馈更新、目标限幅、前馈补偿、PID 闭环、在线保护和 MIT 力矩发送。
  */
#include "yaw_pitch_direct.h"
#include "auto_aim.h"
#include "hwt_imu.h"
#include "bsp_fdcan.h"
#include "detect_task.h"
#include "robot_param.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>

#if (ROBOT_GIMBAL == ROBOT_GIMBAL_YAW_PITCH_DIRECT)

float yaw_can_set_current = 0.0f;
float pitch_can_set_current = 0.0f;
int16_t shoot_can_set_current = 0;

static float yaw_auto_aim_target_vel = 0.0f;
static float pitch_auto_aim_target_vel = 0.0f;

/**
  * @brief          声明 yaw 电机在线检测函数
  * @retval         none
  */
static uint8_t gimbal_yaw_online(void);
/**
  * @brief          声明 pitch MIT 反馈检测函数
  * @retval         none
  */
static uint8_t gimbal_pitch_mit_feedback_ready(void);
/**
  * @brief          声明单轴零输出清理函数
  * @retval         none
  */
static void gimbal_motor_zero_output(gimbal_motor_t *motor);
/**
  * @brief          声明角度归一化函数
  * @retval         归一化后的角度
  */
static float gimbal_wrap_angle(float angle);
/**
  * @brief          声明自瞄补偿读取函数
  * @retval         自瞄补偿量
  */
static float gimbal_take_auto_aim_bias(gimbal_motor_t *motor);
/**
  * @brief          声明浮点限幅函数
  * @retval         限幅后的值
  */
static float gimbal_clamp(float value, float min_value, float max_value);
/**
  * @brief          声明静摩擦补偿原始量计算函数
  * @retval         静摩擦补偿原始量
  */
static float gimbal_calc_static_friction_comp_raw(const gimbal_motor_t *motor, float angle_error);
/**
  * @brief          声明静摩擦补偿更新函数
  * @retval         静摩擦补偿输出
  */
static float gimbal_update_static_friction_comp(gimbal_motor_t *motor, float angle_error);
/**
  * @brief          声明力矩命令限幅函数
  * @retval         力矩命令
  */
static float gimbal_float_to_torque_cmd(float output);
/**
  * @brief          声明惯量前馈计算函数
  * @retval         前馈力矩
  */
static float gimbal_calc_feedforward(gimbal_motor_t *motor);
/**
  * @brief          声明角度反馈力矩计算函数
  * @retval         反馈力矩
  */
static float gimbal_calc_feedback_torque(gimbal_motor_t *motor, gimbal_pid_t *angle_pid, float angle_get, float angle_set);
/**
  * @brief          声明角速度力矩计算函数
  * @retval         控制力矩
  */
static float gimbal_calc_angle_speed_torque(gimbal_motor_t *motor, gimbal_pid_t *pid, float angle_error);
/**
  * @brief          声明自瞄目标速度清理函数
  * @retval         none
  */
static void gimbal_auto_aim_clear_target_vel(gimbal_motor_t *motor);

/**
  * @brief          获取 INS 姿态角数组弱接口
  * @retval         姿态角数组指针
  */
__attribute__((weak)) const float *get_INS_angle_point(void)
{
    return 0;
}

/**
  * @brief          获取陀螺仪角速度数组弱接口
  * @retval         角速度数组指针
  */
__attribute__((weak)) const float *get_gyro_data_point(void)
{
    return 0;
}

/**
  * @brief          获取加速度数组弱接口
  * @retval         加速度数组指针
  */
__attribute__((weak)) const float *get_accel_data_point(void)
{
    return 0;
}

/**
  * @brief          对 MIT 力矩命令执行限幅
  * @retval         限幅后的值
  */
static float gimbal_mit_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/**
  * @brief          将控制器输出转换为 MIT 力矩命令
  * @retval         none
  */
static float gimbal_output_to_mit_torque(float output)
{
    return gimbal_mit_clamp(output, T_MIN, T_MAX);
}

/**
  * @brief          判断 yaw 电机是否在线
  * @retval         none
  */
static uint8_t gimbal_yaw_online(void)
{
    return (uint8_t)(toe_is_error(YAW_GIMBAL_MOTOR_TOE) == 0U);
}

/**
  * @brief          判断 pitch MIT 反馈是否可用
  * @retval         none
  */
static uint8_t gimbal_pitch_mit_feedback_ready(void)
{
    const MITMeasure_t *measure = &MIT_MOTOR_MEASURE[GIMBAL_PITCH_MIT_INDEX];
    uint32_t now = HAL_GetTick();

    return (uint8_t)((toe_is_error(PITCH_GIMBAL_MOTOR_TOE) == 0U) &&
                     (measure->fdb.last_fdb_time != 0U) &&
                     ((uint32_t)(now - measure->fdb.last_fdb_time) <=
                      (uint32_t)GIMBAL_PITCH_MIT_FDB_TIMEOUT));
}

/**
  * @brief          清除单轴前馈轨迹状态
  * @retval         none
  */
static void gimbal_feedforward_clear(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->ref_vel = 0.0f;
    motor->ref_vel_last = 0.0f;
    motor->ref_accel = 0.0f;
    motor->rc_ref_vel = 0.0f;
    motor->rc_ref_vel_last = 0.0f;
    motor->rc_ref_accel = 0.0f;
    motor->rc_ref_target_last = 0.0f;
    motor->rc_ref_target_init = 0u;
    motor->auto_ref_vel = 0.0f;
    motor->auto_ref_vel_last = 0.0f;
    motor->auto_ref_accel = 0.0f;
    motor->auto_ref_target_last = 0.0f;
    motor->auto_ref_target_init = 0u;
    motor->ff_torque = 0.0f;
    gimbal_auto_aim_clear_target_vel(motor);
}

/**
  * @brief          清除单轴控制状态并置零输出
  * @retval         none
  */
static void gimbal_motor_zero_output(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    gimbal_feedforward_clear(motor);
    gimbal_pid_clear(&motor->absolute_angle_pid);
    gimbal_pid_clear(&motor->relative_angle_pid);

    motor->mode = GIMBAL_MOTOR_RAW;
    motor->last_mode = GIMBAL_MOTOR_RAW;
    motor->raw_cmd = 0.0f;
    motor->current_set = 0.0f;
    motor->output = 0.0f;
    motor->given_current = 0.0f;
    motor->pid_torque = 0.0f;
    motor->static_friction_comp = 0.0f;
    motor->rc_control_enable = 0u;
    motor->auto_control_enable = 0u;
    motor->absolute_angle_set = motor->absolute_angle;
    motor->relative_angle_set = motor->relative_angle;
    motor->rc_absolute_angle_set = motor->absolute_angle;
    motor->rc_relative_angle_set = motor->relative_angle;
    motor->auto_absolute_angle_set = motor->absolute_angle;
    motor->auto_relative_angle_set = motor->relative_angle;
    motor->gyro_set = motor->gyro;
}

/**
  * @brief          清除指定控制来源的前馈状态
  * @retval         none
  */
static void gimbal_feedforward_clear_source(gimbal_motor_t *motor,
                                            gimbal_control_source_e source)
{
    if (motor == 0)
    {
        return;
    }

    if (source == GIMBAL_CONTROL_SOURCE_AUTO)
    {
        motor->auto_ref_vel = 0.0f;
        motor->auto_ref_vel_last = 0.0f;
        motor->auto_ref_accel = 0.0f;
        motor->auto_ref_target_last = 0.0f;
        motor->auto_ref_target_init = 0u;
        gimbal_auto_aim_clear_target_vel(motor);
        return;
    }

    motor->rc_ref_vel = 0.0f;
    motor->rc_ref_vel_last = 0.0f;
    motor->rc_ref_accel = 0.0f;
    motor->rc_ref_target_last = 0.0f;
    motor->rc_ref_target_init = 0u;
}

/**
  * @brief          根据目标角跟踪更新前馈速度和加速度
  * @retval         none
  */
static void gimbal_feedforward_track_target(gimbal_motor_t *motor,
                                            float target_angle,
                                            gimbal_control_source_e source)
{
    float ref_vel_cmd;
    float ref_accel_cmd;
    float alpha;
    float accel_limit;
    float *ref_vel;
    float *ref_vel_last;
    float *ref_accel;
    float *target_last;
    uint8_t *target_init;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (motor == 0 || control_dt <= 0.0f)
    {
        return;
    }

    if (source == GIMBAL_CONTROL_SOURCE_AUTO)
    {
        ref_vel = &motor->auto_ref_vel;
        ref_vel_last = &motor->auto_ref_vel_last;
        ref_accel = &motor->auto_ref_accel;
        target_last = &motor->auto_ref_target_last;
        target_init = &motor->auto_ref_target_init;
    }
    else
    {
        ref_vel = &motor->rc_ref_vel;
        ref_vel_last = &motor->rc_ref_vel_last;
        ref_accel = &motor->rc_ref_accel;
        target_last = &motor->rc_ref_target_last;
        target_init = &motor->rc_ref_target_init;
    }

    if (*target_init == 0u)
    {
        *target_last = target_angle;
        *target_init = 1u;
        *ref_vel_last = 0.0f;
        *ref_vel = 0.0f;
        *ref_accel = 0.0f;
        return;
    }

    alpha = gimbal_mit_clamp(YAW_REF_VEL_FILTER_ALPHA, 0.0f, 1.0f);
    ref_vel_cmd = (target_angle - *target_last) / control_dt;

    *ref_vel_last = *ref_vel;
    *ref_vel += alpha * (ref_vel_cmd - *ref_vel);

    ref_accel_cmd = (*ref_vel - *ref_vel_last) / control_dt;
    accel_limit = YAW_REF_ACCEL_LIMIT;
    if (accel_limit > 0.0f)
    {
        ref_accel_cmd = gimbal_mit_clamp(ref_accel_cmd,
                                         -accel_limit,
                                         accel_limit);
    }

    *ref_accel = ref_accel_cmd;
    *target_last = target_angle;
}

/**
  * @brief          将角度归一化到 [-pi, pi]
  * @retval         none
  */
static float yaw_pitch_direct_wrap_angle(float angle)
{
    while (angle > YAW_PITCH_DIRECT_PI)
    {
        angle -= 2.0f * YAW_PITCH_DIRECT_PI;
    }
    while (angle < -YAW_PITCH_DIRECT_PI)
    {
        angle += 2.0f * YAW_PITCH_DIRECT_PI;
    }
    return angle;
}

/**
  * @brief          规划自瞄目标角速度受限的目标角
  * @retval         none
  */
static float gimbal_auto_aim_plan_target(float target_set,
                                         float desired_target,
                                         float *target_vel,
                                         float kp,
                                         float max_speed,
                                         float max_accel,
                                         float dt)
{
    float err;
    float vel_cmd;
    float vel_delta;
    float vel_delta_max;
    float step;

    if (target_vel == 0 || dt <= 0.0f)
    {
        return target_set;
    }

    err = desired_target - target_set;
    vel_cmd = gimbal_mit_clamp(kp * err, -max_speed, max_speed);
    vel_delta_max = max_accel * dt;
    vel_delta = gimbal_mit_clamp(vel_cmd - *target_vel,
                                 -vel_delta_max,
                                  vel_delta_max);
    *target_vel += vel_delta;

    step = *target_vel * dt;
    if ((err > 0.0f && step > err) || (err < 0.0f && step < err))
    {
        step = err;
        *target_vel = 0.0f;
    }

    return target_set + step;
}

/**
  * @brief          清除单轴自瞄目标速度状态
  * @retval         none
  */
static void gimbal_auto_aim_clear_target_vel(gimbal_motor_t *motor)
{
    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        yaw_auto_aim_target_vel = 0.0f;
    }
    else if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        pitch_auto_aim_target_vel = 0.0f;
    }
}

/**
  * @brief          更新 yaw 绝对角目标并执行限幅
  * @retval         none
  */
static void gimbal_yaw_absolute_angle_limit(gimbal_control_t *control,
                                            float add,
                                            uint8_t rc_enable,
                                            uint8_t auto_enable)
{
    gimbal_motor_t *yaw_motor;
    float chassis_yaw;
    float rc_relative_angle_set;
    float auto_relative_angle_set;
    float desired_relative_angle;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (control == 0)
    {
        return;
    }

    yaw_motor = &control->gimbal_yaw_motor;
    chassis_yaw = hwt101_get_yaw_total_rad();
    rc_relative_angle_set =
        yaw_motor->rc_absolute_angle_set -
        chassis_yaw -
        yaw_motor->angle_offset;

    if (rc_enable != 0u)
    {
        rc_relative_angle_set += add;
    }
    else
    {
        rc_relative_angle_set = yaw_motor->relative_angle;
        gimbal_feedforward_clear_source(yaw_motor, GIMBAL_CONTROL_SOURCE_RC);
    }

    rc_relative_angle_set =
        gimbal_mit_clamp(rc_relative_angle_set,
                         yaw_motor->min_relative_angle,
                         yaw_motor->max_relative_angle);
    yaw_motor->rc_relative_angle_set = rc_relative_angle_set;
    yaw_motor->rc_absolute_angle_set =
        chassis_yaw + yaw_motor->angle_offset + rc_relative_angle_set;

    if (auto_enable != 0u)
    {
        desired_relative_angle =
            yaw_motor->relative_angle +
            auto_aim_get_yaw_err_rad();
        desired_relative_angle =
            gimbal_mit_clamp(desired_relative_angle,
                             yaw_motor->min_relative_angle,
                             yaw_motor->max_relative_angle);
        auto_relative_angle_set =
            gimbal_auto_aim_plan_target(yaw_motor->auto_relative_angle_set,
                                        desired_relative_angle,
                                        &yaw_auto_aim_target_vel,
                                        GIMBAL_AUTO_AIM_YAW_KP,
                                        GIMBAL_AUTO_AIM_YAW_MAX_SPEED,
                                        GIMBAL_AUTO_AIM_YAW_MAX_ACCEL,
                                        control_dt);
        auto_relative_angle_set =
            gimbal_mit_clamp(auto_relative_angle_set,
                             yaw_motor->min_relative_angle,
                             yaw_motor->max_relative_angle);
    }
    else
    {
        auto_relative_angle_set = rc_relative_angle_set;
    }

    yaw_motor->auto_relative_angle_set = auto_relative_angle_set;
    yaw_motor->auto_absolute_angle_set =
        chassis_yaw + yaw_motor->angle_offset + auto_relative_angle_set;

    yaw_motor->relative_angle_set =
        rc_relative_angle_set +
        ((auto_enable != 0u) ? (auto_relative_angle_set - yaw_motor->relative_angle) : 0.0f);
    yaw_motor->absolute_angle_set =
        chassis_yaw + yaw_motor->angle_offset + yaw_motor->relative_angle_set;
}

/**
  * @brief          更新 pitch 相对角目标并执行限幅
  * @retval         none
  */
static void gimbal_pitch_relative_angle_limit(gimbal_control_t *control,
                                              float add,
                                              uint8_t rc_enable,
                                              uint8_t auto_enable)
{
    gimbal_motor_t *pitch_motor;
    float rc_relative_angle_set;
    float desired_relative_angle;
    float auto_relative_angle_set;
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (control == 0)
    {
        return;
    }

    pitch_motor = &control->gimbal_pitch_motor;

    if (rc_enable != 0u)
    {
        rc_relative_angle_set = pitch_motor->rc_relative_angle_set + add;
    }
    else
    {
        rc_relative_angle_set = pitch_motor->relative_angle;
        gimbal_feedforward_clear_source(pitch_motor, GIMBAL_CONTROL_SOURCE_RC);
    }

    pitch_motor->rc_relative_angle_set =
        gimbal_mit_clamp(rc_relative_angle_set,
                         pitch_motor->min_relative_angle,
                         pitch_motor->max_relative_angle);
    pitch_motor->rc_absolute_angle_set = pitch_motor->rc_relative_angle_set;

    if (auto_enable != 0u)
    {
        desired_relative_angle =
            pitch_motor->relative_angle +
            auto_aim_get_pitch_err_rad();
        desired_relative_angle =
            gimbal_mit_clamp(desired_relative_angle,
                             pitch_motor->min_relative_angle,
                             pitch_motor->max_relative_angle);
        auto_relative_angle_set =
            gimbal_auto_aim_plan_target(pitch_motor->auto_relative_angle_set,
                                        desired_relative_angle,
                                        &pitch_auto_aim_target_vel,
                                        GIMBAL_AUTO_AIM_PITCH_KP,
                                        GIMBAL_AUTO_AIM_PITCH_MAX_SPEED,
                                        GIMBAL_AUTO_AIM_PITCH_MAX_ACCEL,
                                        control_dt);
        pitch_motor->auto_relative_angle_set =
            gimbal_mit_clamp(auto_relative_angle_set,
                             pitch_motor->min_relative_angle,
                             pitch_motor->max_relative_angle);
    }
    else
    {
        pitch_motor->auto_relative_angle_set = pitch_motor->rc_relative_angle_set;
    }

    pitch_motor->auto_absolute_angle_set = pitch_motor->auto_relative_angle_set;
    pitch_motor->relative_angle_set =
        pitch_motor->rc_relative_angle_set +
        ((auto_enable != 0u) ?
             (pitch_motor->auto_relative_angle_set - pitch_motor->relative_angle) :
             0.0f);
    pitch_motor->absolute_angle_set = pitch_motor->relative_angle_set;
}

/**
  * @brief          更新绝对角目标并执行限幅
  * @retval         none
  */
__attribute__((used)) void gimbal_absolute_angle_limit(gimbal_motor_t *motor, float add)
{
    const float bias = gimbal_take_auto_aim_bias(motor);

    if (motor == 0)
    {
        return;
    }

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        motor->absolute_angle_set += add + bias;
    }
    else
    {
        motor->absolute_angle_set =
            gimbal_clamp(motor->absolute_angle_set + add + bias,
                         motor->min_relative_angle,
                         motor->max_relative_angle);
    }
}

/**
  * @brief          执行云台电机绝对角控制
  * @retval         none
  */
__attribute__((used)) void gimbal_motor_absolute_angle_control(gimbal_motor_t *motor)
{
    float angle_get;
    float angle_set;

    if (motor == 0)
    {
        return;
    }

    motor->ref_vel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_vel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_vel : 0.0f);
    motor->ref_accel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_accel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_accel : 0.0f);

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        angle_get = motor->absolute_angle;
        gimbal_calc_angle_speed_torque(motor,
                                       &motor->absolute_angle_pid,
                                       gimbal_wrap_angle(motor->absolute_angle_set - angle_get));
    }
    else
    {
        angle_get = gimbal_wrap_angle(motor->absolute_angle);
        angle_set = gimbal_wrap_angle(motor->absolute_angle_set);
        gimbal_calc_feedback_torque(motor, &motor->absolute_angle_pid, angle_get, angle_set);
    }

    motor->current_set = motor->pid_torque + gimbal_calc_feedforward(motor);
    motor->output = motor->current_set;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

/**
  * @brief          执行云台电机绝对角控制
  * @retval         none
  */
__attribute__((used)) void gimbal_motor_relative_angle_control(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->ref_vel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_vel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_vel : 0.0f);
    motor->ref_accel =
        ((motor->rc_control_enable != 0u) ? motor->rc_ref_accel : 0.0f) +
        ((motor->auto_control_enable != 0u) ? motor->auto_ref_accel : 0.0f);

    if (motor == &gimbal_control.gimbal_yaw_motor)
    {
        gimbal_calc_feedback_torque(motor,
                                    &motor->relative_angle_pid,
                                    0.0f,
                                    gimbal_wrap_angle(motor->relative_angle_set - motor->relative_angle));
    }
    else
    {
        gimbal_calc_angle_speed_torque(motor,
                                       &motor->relative_angle_pid,
                                       motor->relative_angle_set - motor->relative_angle);
    }

    motor->current_set = motor->pid_torque + gimbal_calc_feedforward(motor);
    motor->output = motor->current_set;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

/**
  * @brief          执行云台电机 RAW 输出控制
  * @retval         none
  */
__attribute__((used)) void gimbal_motor_raw_angle_control(gimbal_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    motor->current_set = motor->raw_cmd;
    motor->output = motor->raw_cmd;
    motor->pid_torque = 0.0f;
    motor->ff_torque = 0.0f;
    motor->static_friction_comp = 0.0f;
    motor->rc_control_enable = 0u;
    motor->auto_control_enable = 0u;
    motor->given_current = gimbal_float_to_torque_cmd(motor->output);
}

/**
  * @brief          初始化云台 PID 控制器
  * @retval         none
  */
__attribute__((used)) void gimbal_pid_init(gimbal_pid_t *pid, float kp, float ki, float kd, float max_out, float max_iout)
{
    float pid_param[3];

    if (pid == NULL)
    {
        return;
    }

    pid_param[0] = kp;
    pid_param[1] = ki;
    pid_param[2] = kd;
    PID_init(pid, PID_POSITION, pid_param, max_out, max_iout);
}

/**
  * @brief          清空云台 PID 控制器状态
  * @retval         none
  */
__attribute__((used)) void gimbal_pid_clear(gimbal_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    PID_clear(pid);
}

/**
  * @brief          计算云台 PID 输出
  * @retval         none
  */
__attribute__((used)) float gimbal_pid_calc(gimbal_pid_t *pid, float get, float set, float error_delta)
{
    (void)error_delta;

    if (pid == NULL)
    {
        return 0.0f;
    }

    return PID_Calc(pid, get, set);
}

/**
  * @brief          将角度归一化到 [-pi, pi]
  * @retval         none
  */
static float gimbal_wrap_angle(float angle)
{
    while (angle > GIMBAL_PI)
    {
        angle -= 2.0f * GIMBAL_PI;
    }
    while (angle < -GIMBAL_PI)
    {
        angle += 2.0f * GIMBAL_PI;
    }
    return angle;
}

/**
  * @brief          读取并清除单轴自瞄角度补偿量
  * @retval         none
  */
static float gimbal_take_auto_aim_bias(gimbal_motor_t *motor)
{
    float bias = 0.0f;

    (void)motor;

    return bias;
}

/**
  * @brief          对浮点数执行区间限幅
  * @retval         none
  */
static float gimbal_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/**
  * @brief          计算单轴静摩擦补偿原始量
  * @retval         静摩擦补偿原始量
  */
static float gimbal_calc_static_friction_comp_raw(const gimbal_motor_t *motor, float angle_error)
{
    float abs_error;
    float comp_scale;
    float deadband;
    float fullband;

    if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        deadband = GIMBAL_PITCH_STATIC_FRICTION_DEADBAND;
        fullband = GIMBAL_PITCH_STATIC_FRICTION_FULLBAND;
    }
    else
    {
        deadband = GIMBAL_YAW_STATIC_FRICTION_DEADBAND;
        fullband = GIMBAL_YAW_STATIC_FRICTION_FULLBAND;
    }

    abs_error = fabsf(angle_error);
    if (abs_error <= deadband)
    {
        return 0.0f;
    }

    if (fullband <= deadband ||
        abs_error >= fullband)
    {
        comp_scale = 1.0f;
    }
    else
    {
        comp_scale =
            (abs_error - deadband) /
            (fullband - deadband);
    }

    if (angle_error > 0.0f)
    {
        if (motor == &gimbal_control.gimbal_pitch_motor)
        {
            return GIMBAL_PITCH_STATIC_FRICTION_COMP_UP * comp_scale;
        }
        return GIMBAL_YAW_STATIC_FRICTION_COMP * comp_scale;
    }
    if (angle_error < 0.0f)
    {
        if (motor == &gimbal_control.gimbal_pitch_motor)
        {
            return -GIMBAL_PITCH_STATIC_FRICTION_COMP_DOWN * comp_scale;
        }
        return -GIMBAL_YAW_STATIC_FRICTION_COMP * comp_scale;
    }
    return 0.0f;
}

/**
  * @brief          更新单轴静摩擦补偿输出
  * @retval         静摩擦补偿输出
  */
static float gimbal_update_static_friction_comp(gimbal_motor_t *motor, float angle_error)
{
    float alpha;
    float comp_cmd;

    comp_cmd = gimbal_calc_static_friction_comp_raw(motor, angle_error);
    if (motor == 0)
    {
        return comp_cmd;
    }

    if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        alpha = gimbal_clamp(GIMBAL_PITCH_STATIC_FRICTION_FILTER_ALPHA, 0.0f, 1.0f);
    }
    else
    {
        alpha = gimbal_clamp(GIMBAL_YAW_STATIC_FRICTION_FILTER_ALPHA, 0.0f, 1.0f);
    }
    motor->static_friction_comp +=
        alpha * (comp_cmd - motor->static_friction_comp);

    return motor->static_friction_comp;
}

/**
  * @brief          将浮点输出限制为力矩命令范围
  * @retval         力矩命令
  */
static float gimbal_float_to_torque_cmd(float output)
{
    return gimbal_clamp(output, T_MIN, T_MAX);
}

/**
  * @brief          计算单轴惯量前馈力矩
  * @retval         前馈力矩
  */
static float gimbal_calc_feedforward(gimbal_motor_t *motor)
{
    float velocity_torque = 0.0f;

    if (motor == 0)
    {
        return 0.0f;
    }

    if (motor == &gimbal_control.gimbal_pitch_motor)
    {
        velocity_torque = PITCH_VELOCITY_FF_GAIN * motor->ref_vel;
    }

    motor->ff_torque = velocity_torque + motor->inertia_kgm2 * motor->ref_accel;

    return motor->ff_torque;
}

/**
  * @brief          计算单轴角度反馈力矩
  * @retval         反馈力矩
  */
static float gimbal_calc_feedback_torque(gimbal_motor_t *motor, gimbal_pid_t *angle_pid, float angle_get, float angle_set)
{
    float angle_torque;
    float angle_error;

    if (motor == 0 || angle_pid == 0)
    {
        return 0.0f;
    }

    motor->gyro_set = motor->ref_vel;
    angle_error = angle_set - angle_get;
    angle_torque = gimbal_pid_calc(angle_pid, angle_get, angle_set, 0.0f);
    motor->pid_torque =
        gimbal_clamp(angle_torque + gimbal_update_static_friction_comp(motor, angle_error),
                     -angle_pid->max_out,
                     angle_pid->max_out);

    return motor->pid_torque;
}

/**
  * @brief          根据角度误差和角速度误差计算控制力矩
  * @retval         控制力矩
  */
static float gimbal_calc_angle_speed_torque(gimbal_motor_t *motor, gimbal_pid_t *pid, float angle_error)
{
    float speed_error;
    float output;

    if (motor == 0 || pid == 0)
    {
        return 0.0f;
    }

    motor->gyro_set = motor->ref_vel;
    speed_error = motor->gyro_set - motor->gyro;

    pid->set = angle_error;
    pid->fdb = 0.0f;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = angle_error;
    pid->Pout = pid->Kp * angle_error;
    pid->Iout = 0.0f;
    pid->Dout = pid->Kd * speed_error;

    output = pid->Pout + pid->Dout + gimbal_update_static_friction_comp(motor, angle_error);
    output = gimbal_clamp(output, -pid->max_out, pid->max_out);
    pid->out = output;
    motor->pid_torque = output;

    return motor->pid_torque;
}

/**
  * @brief          清空云台两轴全部 PID 状态
  * @retval         none
  */
static void gimbal_total_pid_clear(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    gimbal_pid_clear(&control->gimbal_yaw_motor.absolute_angle_pid);
    gimbal_pid_clear(&control->gimbal_yaw_motor.relative_angle_pid);
    gimbal_pid_clear(&control->gimbal_pitch_motor.absolute_angle_pid);
    gimbal_pid_clear(&control->gimbal_pitch_motor.relative_angle_pid);
}

/**
  * @brief          初始化云台控制结构体
  * @retval         none
  */
__attribute__((used)) void gimbal_init(gimbal_control_t *control)
{
    gravity_comp_param_t gravity_comp_param;

    if (control == 0)
    {
        return;
    }

    memset(control, 0, sizeof(*control));

    control->gimbal_rc_ctrl = get_remote_control_point();
    control->gimbal_INT_angle_point = get_INS_angle_point();
    control->gimbal_INT_gyro_point = get_gyro_data_point();
    control->gimbal_INT_accel_point = get_accel_data_point();

    control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
    control->gimbal_yaw_motor.last_mode = GIMBAL_MOTOR_RAW;
    control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
    control->gimbal_pitch_motor.last_mode = GIMBAL_MOTOR_RAW;

    gimbal_pid_init(&control->gimbal_yaw_motor.absolute_angle_pid,
                    YAW_GYRO_ABSOLUTE_PID_KP,
                    YAW_GYRO_ABSOLUTE_PID_KI,
                    YAW_GYRO_ABSOLUTE_PID_KD,
                    YAW_GYRO_ABSOLUTE_PID_MAX_OUT,
                    YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
    gimbal_pid_init(&control->gimbal_yaw_motor.relative_angle_pid,
                    YAW_ENCODE_RELATIVE_PID_KP,
                    YAW_ENCODE_RELATIVE_PID_KI,
                    YAW_ENCODE_RELATIVE_PID_KD,
                    YAW_ENCODE_RELATIVE_PID_MAX_OUT,
                    YAW_ENCODE_RELATIVE_PID_MAX_IOUT);

    gimbal_pid_init(&control->gimbal_pitch_motor.absolute_angle_pid,
                    PITCH_GYRO_ABSOLUTE_PID_KP,
                    PITCH_GYRO_ABSOLUTE_PID_KI,
                    PITCH_GYRO_ABSOLUTE_PID_KD,
                    PITCH_GYRO_ABSOLUTE_PID_MAX_OUT,
                    PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
    gimbal_pid_init(&control->gimbal_pitch_motor.relative_angle_pid,
                    PITCH_ENCODE_RELATIVE_PID_KP,
                    PITCH_ENCODE_RELATIVE_PID_KI,
                    PITCH_ENCODE_RELATIVE_PID_KD,
                    PITCH_ENCODE_RELATIVE_PID_MAX_OUT,
                    PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);

    control->gimbal_yaw_motor.max_relative_angle = YAW_MAX_RELATIVE_ANGLE;
    control->gimbal_yaw_motor.min_relative_angle = YAW_MIN_RELATIVE_ANGLE;
    control->gimbal_pitch_motor.max_relative_angle = PITCH_MAX_RELATIVE_ANGLE;
    control->gimbal_pitch_motor.min_relative_angle = PITCH_MIN_RELATIVE_ANGLE;
    control->gimbal_yaw_motor.inertia_kgm2 = YAW_INERTIA_KGM2;
    control->gimbal_pitch_motor.inertia_kgm2 = PITCH_INERTIA_KGM2;
		
//		Motor_MIT_MODE(&hfdcan1, DM_YAW_CAN_ID);
    Motor_ENABLE(&hfdcan1, DM_YAW_CAN_ID);
//		Motor_MIT_MODE(&hfdcan2, DM_PIT_CAN_ID);
    Motor_ENABLE(&hfdcan2, DM_PIT_CAN_ID);

    gimbal_total_pid_clear(control);
    gimbal_feedback_update(control);

    control->gimbal_yaw_motor.absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.rc_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.rc_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.auto_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
    control->gimbal_yaw_motor.auto_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
    control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;

    control->gimbal_pitch_motor.absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.rc_absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.rc_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.auto_absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
    control->gimbal_pitch_motor.auto_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
    control->gimbal_pitch_motor.gyro_set = control->gimbal_pitch_motor.gyro;

    gravity_comp_param.mass_kg = PITCH_GRAVITY_COMP_MASS_KG;
    gravity_comp_param.com_forward_m = PITCH_GRAVITY_COMP_COM_FORWARD_M;
    gravity_comp_param.com_up_m = PITCH_GRAVITY_COMP_COM_UP_M;
    gravity_comp_param.gravity_mps2 = GRAVITY_COMP_DEFAULT_GRAVITY;
    control->gimbal_pitch_gravity_comp.output_scale = OUTPUT_SCALE;
    control->gimbal_pitch_gravity_comp.output_limit = PITCH_GRAVITY_COMP_OUTPUT_LIMIT;
    gravity_comp_init(&control->gimbal_pitch_gravity_comp, &gravity_comp_param);
}

/**
  * @brief          设置云台控制模式
  * @retval         none
  */
__attribute__((used)) void gimbal_set_mode(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    gimbal_behaviour_mode_set(control);
    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
    }
}

/**
  * @brief          更新云台反馈数据
  * @retval         none
  */
__attribute__((used)) void gimbal_feedback_update(gimbal_control_t *control)
{
    float chassis_yaw = 0.0f;
    float yaw_relative = 0.0f;
    float pitch_motor_pos = 0.0f;
    float yaw_gyro_last = 0.0f;
    float pitch_gyro_last = 0.0f;
    float relative_speed_cmd = 0.0f;
    float relative_speed_alpha = 0.0f;
    uint8_t pitch_feedback_ready = gimbal_pitch_mit_feedback_ready();
    const float *imu_gyro = hwt906_get_gimbal_gyro_point();
    const float control_dt = (float)GIMBAL_CONTROL_TIME * 0.001f;

    if (control == 0)
    {
        return;
    }

    if (control->gimbal_INT_angle_point != 0)
    {
        control->gimbal_yaw_motor.absolute_angle =
            control->gimbal_INT_angle_point[INS_YAW_ADDRESS_OFFSET];
        if (pitch_feedback_ready != 0u)
        {
            pitch_motor_pos = MIT_MOTOR_MEASURE[GIMBAL_PITCH_MIT_INDEX].fdb.pos;
        }

        chassis_yaw = hwt101_get_yaw_total_rad();

        if (control->gimbal_yaw_motor.angle_offset_init == 0u)
        {
            control->gimbal_yaw_motor.angle_offset = 0.0f;

            control->gimbal_yaw_motor.relative_angle = 0.0f;
            control->gimbal_yaw_motor.relative_angle_set = 0.0f;
            control->gimbal_yaw_motor.rc_relative_angle_set = 0.0f;
            control->gimbal_yaw_motor.auto_relative_angle_set = 0.0f;
            control->gimbal_yaw_motor.absolute_angle_set =
                control->gimbal_yaw_motor.absolute_angle;
            control->gimbal_yaw_motor.rc_absolute_angle_set =
                control->gimbal_yaw_motor.absolute_angle;
            control->gimbal_yaw_motor.auto_absolute_angle_set =
                control->gimbal_yaw_motor.absolute_angle;
            control->gimbal_yaw_motor.angle_offset_init = 1u;
        }
        else
        {
            yaw_relative =
                control->gimbal_yaw_motor.absolute_angle -
                chassis_yaw -
                control->gimbal_yaw_motor.angle_offset;

            control->gimbal_yaw_motor.relative_angle =
                yaw_pitch_direct_wrap_angle(yaw_relative);
        }

        if (pitch_feedback_ready == 0u)
        {
            control->gimbal_pitch_motor.relative_speed = 0.0f;
            control->gimbal_pitch_motor.relative_speed_update_init = 0u;
        }
        else if (control->gimbal_pitch_motor.angle_offset_init == 0u)
        {
            control->gimbal_pitch_motor.angle_offset =
                pitch_motor_pos;

            control->gimbal_pitch_motor.absolute_angle = 0.0f;
            control->gimbal_pitch_motor.relative_angle = 0.0f;
            control->gimbal_pitch_motor.relative_angle_set = 0.0f;
            control->gimbal_pitch_motor.rc_relative_angle_set = 0.0f;
            control->gimbal_pitch_motor.auto_relative_angle_set = 0.0f;
            control->gimbal_pitch_motor.relative_angle_last = 0.0f;
            control->gimbal_pitch_motor.relative_speed = 0.0f;
            control->gimbal_pitch_motor.relative_speed_update_init = 1u;
            control->gimbal_pitch_motor.absolute_angle_set =
                control->gimbal_pitch_motor.absolute_angle;
            control->gimbal_pitch_motor.rc_absolute_angle_set =
                control->gimbal_pitch_motor.absolute_angle;
            control->gimbal_pitch_motor.auto_absolute_angle_set =
                control->gimbal_pitch_motor.absolute_angle;
            control->gimbal_pitch_motor.angle_offset_init = 1u;
        }
        else
        {
            /* MIT pitch 电机正方向与机械 pitch 正方向相同：
             * 上电机械零位为 0，按右手系 pitch 正方向计角。
             */
            control->gimbal_pitch_motor.absolute_angle =
                pitch_motor_pos -
                control->gimbal_pitch_motor.angle_offset;
            control->gimbal_pitch_motor.relative_angle =
                control->gimbal_pitch_motor.absolute_angle;

            if (control->gimbal_pitch_motor.relative_speed_update_init == 0u)
            {
                control->gimbal_pitch_motor.relative_angle_last =
                    control->gimbal_pitch_motor.relative_angle;
                control->gimbal_pitch_motor.relative_speed = 0.0f;
                control->gimbal_pitch_motor.relative_speed_update_init = 1u;
            }
            else
            {
                relative_speed_cmd =
                    (control->gimbal_pitch_motor.relative_angle -
                     control->gimbal_pitch_motor.relative_angle_last) / control_dt;
                relative_speed_alpha =
                    gimbal_mit_clamp(PITCH_RELATIVE_SPEED_FILTER_ALPHA, 0.0f, 1.0f);

                control->gimbal_pitch_motor.relative_speed +=
                    relative_speed_alpha *
                    (relative_speed_cmd - control->gimbal_pitch_motor.relative_speed);
                control->gimbal_pitch_motor.relative_angle_last =
                    control->gimbal_pitch_motor.relative_angle;
            }
        }
    }

    if (control->gimbal_INT_gyro_point != 0)
    {
        yaw_gyro_last = control->gimbal_yaw_motor.gyro;
        pitch_gyro_last = control->gimbal_pitch_motor.gyro;

        control->gimbal_yaw_motor.gyro =
            control->gimbal_INT_gyro_point[INS_GYRO_Z_ADDRESS_OFFSET];
        control->gimbal_pitch_motor.gyro =
            imu_gyro[HWT_AXIS_PITCH];

        if (control->gimbal_yaw_motor.gyro_update_init == 0u)
        {
            control->gimbal_yaw_motor.gyro_last = control->gimbal_yaw_motor.gyro;
            control->gimbal_yaw_motor.gyro_accel = 0.0f;
            control->gimbal_yaw_motor.gyro_update_init = 1u;
        }
        else
        {
            control->gimbal_yaw_motor.gyro_last = yaw_gyro_last;
            control->gimbal_yaw_motor.gyro_accel =
                (control->gimbal_yaw_motor.gyro - yaw_gyro_last) / control_dt;
        }

        if (control->gimbal_pitch_motor.gyro_update_init == 0u)
        {
            control->gimbal_pitch_motor.gyro_last = control->gimbal_pitch_motor.gyro;
            control->gimbal_pitch_motor.gyro_accel = 0.0f;
            control->gimbal_pitch_motor.gyro_update_init = 1u;
        }
        else
        {
            control->gimbal_pitch_motor.gyro_last = pitch_gyro_last;
            control->gimbal_pitch_motor.gyro_accel =
                (control->gimbal_pitch_motor.gyro - pitch_gyro_last) / control_dt;
        }
    }
}

/**
  * @brief          处理云台模式切换过渡
  * @retval         none
  */
__attribute__((used)) void gimbal_mode_change_control_transit(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_RAW && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW)
    {
        control->gimbal_yaw_motor.raw_cmd = control->gimbal_yaw_motor.current_set = control->gimbal_yaw_motor.given_current;
    }
    else if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_GYRO && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        control->gimbal_yaw_motor.absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
        control->gimbal_yaw_motor.rc_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
        control->gimbal_yaw_motor.auto_absolute_angle_set = control->gimbal_yaw_motor.absolute_angle;
        control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;
    }
    else if (control->gimbal_yaw_motor.last_mode != GIMBAL_MOTOR_ENCODE && control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        control->gimbal_yaw_motor.relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.rc_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.auto_relative_angle_set = control->gimbal_yaw_motor.relative_angle;
        control->gimbal_yaw_motor.gyro_set = control->gimbal_yaw_motor.gyro;
    }
    control->gimbal_yaw_motor.last_mode = control->gimbal_yaw_motor.mode;

    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
        return;
    }

    if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_RAW && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
    {
        control->gimbal_pitch_motor.raw_cmd = control->gimbal_pitch_motor.current_set = control->gimbal_pitch_motor.given_current;
    }
    else if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_GYRO && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        control->gimbal_pitch_motor.absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
        control->gimbal_pitch_motor.rc_absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
        control->gimbal_pitch_motor.auto_absolute_angle_set = control->gimbal_pitch_motor.absolute_angle;
        control->gimbal_pitch_motor.gyro_set = control->gimbal_pitch_motor.gyro;
    }
    else if (control->gimbal_pitch_motor.last_mode != GIMBAL_MOTOR_ENCODE && control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        control->gimbal_pitch_motor.relative_angle_set = control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.rc_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.auto_relative_angle_set = control->gimbal_pitch_motor.relative_angle;
        control->gimbal_pitch_motor.gyro_set = control->gimbal_pitch_motor.gyro;
    }

    control->gimbal_pitch_motor.last_mode = control->gimbal_pitch_motor.mode;
}

/**
  * @brief          生成云台目标控制量
  * @retval         none
  */
__attribute__((used)) void gimbal_set_control(gimbal_control_t *control)
{
    static float add_yaw = 0.0f;
    static float add_pitch = 0.0f;
    uint8_t auto_enable;
    uint8_t yaw_rc_enable;
    uint8_t pitch_rc_enable;
    gimbal_motor_t *yaw_motor;
    gimbal_motor_t *pitch_motor;

    if (control == 0)
    {
        return;
    }

    gimbal_behaviour_control_set(&add_yaw, &add_pitch, control);
    auto_enable = auto_aim_is_active();
    yaw_rc_enable =
        (uint8_t)((auto_enable == 0u) || (add_yaw != 0.0f));
    pitch_rc_enable =
        (uint8_t)((auto_enable == 0u) || (add_pitch != 0.0f));
    yaw_motor = &control->gimbal_yaw_motor;
    pitch_motor = &control->gimbal_pitch_motor;

    if (yaw_motor->mode == GIMBAL_MOTOR_RAW)
    {
        auto_aim_reset_delta_accum();
        gimbal_feedforward_clear(yaw_motor);
        yaw_motor->rc_control_enable = 0u;
        yaw_motor->auto_control_enable = 0u;
        yaw_motor->raw_cmd = add_yaw;
    }
    else if (yaw_motor->mode == GIMBAL_MOTOR_GYRO)
    {
        yaw_motor->rc_control_enable = yaw_rc_enable;
        yaw_motor->auto_control_enable = auto_enable;
        gimbal_yaw_absolute_angle_limit(control,
                                        add_yaw,
                                        yaw_rc_enable,
                                        auto_enable);
        if (yaw_rc_enable != 0u)
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->rc_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->auto_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
        else
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->auto_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
    }
    else if (yaw_motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        yaw_motor->rc_control_enable = yaw_rc_enable;
        yaw_motor->auto_control_enable = auto_enable;
        if (yaw_rc_enable != 0u)
        {
            yaw_motor->rc_relative_angle_set += add_yaw;
        }
        else
        {
            yaw_motor->rc_relative_angle_set = yaw_motor->relative_angle;
            gimbal_feedforward_clear_source(yaw_motor, GIMBAL_CONTROL_SOURCE_RC);
        }
        yaw_motor->rc_relative_angle_set =
            gimbal_mit_clamp(yaw_motor->rc_relative_angle_set,
                             yaw_motor->min_relative_angle,
                             yaw_motor->max_relative_angle);
        if (auto_enable != 0u)
        {
            yaw_motor->auto_relative_angle_set =
                gimbal_auto_aim_plan_target(yaw_motor->auto_relative_angle_set,
                                            yaw_motor->relative_angle + auto_aim_get_yaw_err_rad(),
                                            &yaw_auto_aim_target_vel,
                                            GIMBAL_AUTO_AIM_YAW_KP,
                                            GIMBAL_AUTO_AIM_YAW_MAX_SPEED,
                                            GIMBAL_AUTO_AIM_YAW_MAX_ACCEL,
                                            (float)GIMBAL_CONTROL_TIME * 0.001f);
            yaw_motor->auto_relative_angle_set =
                gimbal_mit_clamp(yaw_motor->auto_relative_angle_set,
                                 yaw_motor->min_relative_angle,
                                 yaw_motor->max_relative_angle);
        }
        else
        {
            yaw_motor->auto_relative_angle_set = yaw_motor->rc_relative_angle_set;
        }
        yaw_motor->relative_angle_set =
            yaw_motor->rc_relative_angle_set +
            ((auto_enable != 0u) ?
                 (yaw_motor->auto_relative_angle_set - yaw_motor->relative_angle) :
                 0.0f);
        if (yaw_rc_enable != 0u)
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->rc_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->auto_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
        else
        {
            gimbal_feedforward_track_target(yaw_motor,
                                            yaw_motor->auto_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
    }

    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        gimbal_motor_zero_output(pitch_motor);
        return;
    }

    if (pitch_motor->mode == GIMBAL_MOTOR_RAW)
    {
        auto_aim_reset_delta_accum();
        gimbal_feedforward_clear(pitch_motor);
        pitch_motor->rc_control_enable = 0u;
        pitch_motor->auto_control_enable = 0u;
        pitch_motor->raw_cmd = add_pitch;
    }
    else if (pitch_motor->mode == GIMBAL_MOTOR_GYRO)
    {
        pitch_motor->rc_control_enable = pitch_rc_enable;
        pitch_motor->auto_control_enable = auto_enable;
        if (pitch_rc_enable != 0u)
        {
            pitch_motor->rc_absolute_angle_set =
                gimbal_mit_clamp(pitch_motor->rc_absolute_angle_set + add_pitch,
                                 pitch_motor->min_relative_angle,
                                 pitch_motor->max_relative_angle);
        }
        else
        {
            pitch_motor->rc_absolute_angle_set = pitch_motor->absolute_angle;
            gimbal_feedforward_clear_source(pitch_motor, GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            pitch_motor->auto_absolute_angle_set =
                gimbal_auto_aim_plan_target(pitch_motor->auto_absolute_angle_set,
                                            pitch_motor->absolute_angle + auto_aim_get_pitch_err_rad(),
                                            &pitch_auto_aim_target_vel,
                                            GIMBAL_AUTO_AIM_PITCH_KP,
                                            GIMBAL_AUTO_AIM_PITCH_MAX_SPEED,
                                            GIMBAL_AUTO_AIM_PITCH_MAX_ACCEL,
                                            (float)GIMBAL_CONTROL_TIME * 0.001f);
            pitch_motor->auto_absolute_angle_set =
                gimbal_mit_clamp(pitch_motor->auto_absolute_angle_set,
                                 pitch_motor->min_relative_angle,
                                 pitch_motor->max_relative_angle);
        }
        else
        {
            pitch_motor->auto_absolute_angle_set = pitch_motor->rc_absolute_angle_set;
        }
        pitch_motor->absolute_angle_set =
            pitch_motor->rc_absolute_angle_set +
            ((auto_enable != 0u) ?
                 (pitch_motor->auto_absolute_angle_set - pitch_motor->absolute_angle) :
                 0.0f);
        if (pitch_rc_enable != 0u)
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->rc_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->auto_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
        else
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->auto_absolute_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
    }
    else if (pitch_motor->mode == GIMBAL_MOTOR_ENCODE)
    {
        pitch_motor->rc_control_enable = pitch_rc_enable;
        pitch_motor->auto_control_enable = auto_enable;
        gimbal_pitch_relative_angle_limit(control,
                                          add_pitch,
                                          pitch_rc_enable,
                                          auto_enable);
        if (pitch_rc_enable != 0u)
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->rc_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_RC);
        }
        if (auto_enable != 0u)
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->auto_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
        else
        {
            gimbal_feedforward_track_target(pitch_motor,
                                            pitch_motor->auto_relative_angle_set,
                                            GIMBAL_CONTROL_SOURCE_AUTO);
        }
    }
}

/**
  * @brief          生成云台目标控制量
  * @retval         none
  */
__attribute__((used)) void gimbal_control_loop(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    if ((control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW) ||
        (gimbal_yaw_online() == 0u))
    {
        /* RAW 无力模式和 yaw 离线保护直接零输出，不进入 yaw 闭环。 */
        gimbal_motor_zero_output(&control->gimbal_yaw_motor);
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control->gimbal_yaw_motor);
    }
    else if (control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_motor_relative_angle_control(&control->gimbal_yaw_motor);
    }

    if (gimbal_pitch_mit_feedback_ready() == 0u)
    {
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
        return;
    }

    if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
    {
        /* RAW 无力模式直接零输出，不进入 pitch 闭环。 */
        gimbal_motor_zero_output(&control->gimbal_pitch_motor);
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control->gimbal_pitch_motor);
    }
    else if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_ENCODE)
    {
        gimbal_motor_relative_angle_control(&control->gimbal_pitch_motor);
    }
}

/**
  * @brief          发送云台电机命令
  * @retval         none
  */
__attribute__((used)) void gimbal_send_cmd(gimbal_control_t *control)
{
    float yaw_cmd_torque;
    float pitch_cmd_torque;

    if (control == 0)
    {
        return;
    }

    if ((control->gimbal_yaw_motor.mode == GIMBAL_MOTOR_RAW) ||
        (gimbal_yaw_online() == 0u))
    {
        yaw_can_set_current = 0.0f;
    }
    else
    {
        yaw_can_set_current = control->gimbal_yaw_motor.given_current;
    }

    if ((control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW) ||
        (gimbal_pitch_mit_feedback_ready() == 0u))
    {
        pitch_can_set_current = 0.0f;
    }
    else
    {
        pitch_can_set_current = control->gimbal_pitch_motor.given_current;
    }

    yaw_cmd_torque = gimbal_output_to_mit_torque(yaw_can_set_current);
    pitch_cmd_torque = gimbal_output_to_mit_torque(pitch_can_set_current);

    CAN_cmd_MIT(&hfdcan1, DM_YAW_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, yaw_cmd_torque);
    CAN_cmd_MIT(&hfdcan2, DM_PIT_CAN_ID, 0.0f, 0.0f, 0.0f, 0.0f, pitch_cmd_torque);
}
#endif
