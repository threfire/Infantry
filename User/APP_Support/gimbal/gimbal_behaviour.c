/**
  * @file       gimbal_behaviour.c
  * @brief      云台行为模式控制
  * @note       将遥控器、键鼠、自瞄和初始化状态转换为云台行为模式与角度增量。
  */
#include "gimbal_behaviour.h"
#include <math.h>

volatile gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
  * @brief          对遥控器通道值执行死区处理
  * @retval         死区处理后的通道值
  */
static int16_t gimbal_apply_deadband(int16_t value)
{
    if (value > RC_DEADBAND || value < -RC_DEADBAND)
    {
        return value;
    }
    return 0;
}

/**
  * @brief          读取遥控器和鼠标输入并转换为云台角度增量
  * @retval         none
  */
static void gimbal_read_manual_input(float *yaw, float *pitch, gimbal_control_t *control)
{
    int16_t yaw_channel = 0;
    int16_t pitch_channel = 0;
    int mouse_x = 0;
    int mouse_y = 0;

    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;

    if (control->gimbal_rc_ctrl == 0)
    {
        return;
    }

    yaw_channel = gimbal_apply_deadband(control->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL]);
    pitch_channel = gimbal_apply_deadband(control->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL]);
    mouse_x = control->gimbal_rc_ctrl->mouse.x;
    mouse_y = control->gimbal_rc_ctrl->mouse.y;

    *yaw = yaw_channel * YAW_RC_SEN - (float)mouse_x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + (float)mouse_y * PITCH_MOUSE_SEN;
}

/**
  * @brief          设置云台行为模式对应的电机控制模式
  * @retval         none
  */
void gimbal_behaviour_mode_set(gimbal_control_t *control)
{
    if (control == 0)
    {
        return;
    }

    gimbal_behavour_set(control);

    switch (gimbal_behaviour)
    {
    case GIMBAL_ZERO_FORCE:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
        break;

    case GIMBAL_INIT:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_GYRO;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_CALI:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
        break;

    case GIMBAL_ABSOLUTE_ANGLE:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_GYRO;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_RELATIVE_ANGLE:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_GYRO;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_SPIN:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_GYRO;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_ENCODE;
        break;

    case GIMBAL_MOTIONLESS:
    default:
        control->gimbal_yaw_motor.mode = GIMBAL_MOTOR_RAW;
        control->gimbal_pitch_motor.mode = GIMBAL_MOTOR_RAW;
        break;
    }
}

/**
  * @brief          根据云台行为模式生成两轴角度增量
  * @retval         none
  */
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *control)
{
    if (add_yaw == 0 || add_pitch == 0 || control == 0)
    {
        return;
    }

    switch (gimbal_behaviour)
    {
    case GIMBAL_ZERO_FORCE:
        gimbal_zero_force_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_INIT:
        gimbal_init_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_CALI:
        gimbal_cali_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_ABSOLUTE_ANGLE:
        gimbal_absolute_angle_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_RELATIVE_ANGLE:
        gimbal_relative_angle_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_MOTIONLESS:
        gimbal_motionless_control(add_yaw, add_pitch, control);
        break;
    case GIMBAL_SPIN:
        gimbal_spin_control(add_yaw, add_pitch, control);
        break;
    default:
        *add_yaw = 0.0f;
        *add_pitch = 0.0f;
        break;
    }
}

/**
  * @brief          输出云台对底盘停止的联动请求
  * @retval         none
  */
bool gimbal_cmd_to_chassis_stop(void)
{
    return (gimbal_behaviour == GIMBAL_INIT ||
            gimbal_behaviour == GIMBAL_CALI ||
            gimbal_behaviour == GIMBAL_MOTIONLESS ||
            gimbal_behaviour == GIMBAL_ZERO_FORCE);
}

/**
  * @brief          输出云台对底盘停止的联动请求
  * @retval         none
  */
bool gimbal_cmd_to_shoot_stop(void)
{
    return (gimbal_behaviour == GIMBAL_INIT ||
            gimbal_behaviour == GIMBAL_CALI ||
            gimbal_behaviour == GIMBAL_ZERO_FORCE);
}

/**
  * @brief          更新云台行为状态机
  * @retval         none
  */
void gimbal_behavour_set(gimbal_control_t *control)
{
    static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
    static unsigned int init_time = 0U;
    static unsigned int init_stop_time = 0U;
    int mode_switch = 0;

    if (control == 0)
    {
        return;
    }

    if (control->gimbal_rc_ctrl == 0)
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
        last_gimbal_behaviour = gimbal_behaviour;
        return;
    }

    if (gimbal_behaviour == GIMBAL_CALI &&
        control->gimbal_cali.step != 0U &&
        control->gimbal_cali.step != GIMBAL_CALI_END_STEP)
    {
        return;
    }

    if (control->gimbal_cali.step == GIMBAL_CALI_START_STEP)
    {
        gimbal_behaviour = GIMBAL_CALI;
        last_gimbal_behaviour = gimbal_behaviour;
        return;
    }

    if (gimbal_behaviour == GIMBAL_INIT)
    {
        if (fabsf(control->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
            fabsf(control->gimbal_pitch_motor.relative_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR)
        {
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {
            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME)
        {
            last_gimbal_behaviour = gimbal_behaviour;
            return;
        }

        init_time = 0U;
        init_stop_time = 0U;
    }

    mode_switch = control->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL];

    if (switch_is_mid(mode_switch))
    {
        gimbal_behaviour = GIMBAL_MOTIONLESS;
    }
    else if (switch_is_up(mode_switch))
    {
        gimbal_behaviour = GIMBAL_SPIN;
    }
    else if (switch_is_down(mode_switch))
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }
    else
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }

    if (switch_is_down(mode_switch) && (control->gimbal_rc_ctrl->key.v & GIMBAL_ZERO_KEYBOARD))
    {
        gimbal_behaviour = GIMBAL_MOTIONLESS;
    }
    else if (switch_is_down(mode_switch) && (control->gimbal_rc_ctrl->key.v & GIMBAL_SPIN_KEYBOARD))
    {
        gimbal_behaviour = GIMBAL_SPIN;
    }
    else if (switch_is_down(mode_switch) && (control->gimbal_rc_ctrl->key.v & GIMBAL_RELATIVE_KEYBOARD))
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }

    if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE &&
        gimbal_behaviour != GIMBAL_ZERO_FORCE)
    {
        gimbal_behaviour = GIMBAL_INIT;
    }

    last_gimbal_behaviour = gimbal_behaviour;
}

/**
  * @brief          无力模式云台角度增量清零
  * @retval         none
  */
void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    (void)control;

    if (yaw == 0 || pitch == 0)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          初始化模式云台回中控制量生成
  * @retval         none
  */
void gimbal_init_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    *yaw = (INIT_YAW_SET - control->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    *pitch = (INIT_PITCH_SET - control->gimbal_pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
}

/**
  * @brief          云台校准模式控制量生成
  * @retval         none
  */
void gimbal_cali_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    switch (control->gimbal_cali.step)
    {
    case GIMBAL_CALI_PITCH_MAX_STEP:
        *yaw = 0.0f;
        *pitch = GIMBAL_CALI_MOTOR_SET;
        break;
    case GIMBAL_CALI_PITCH_MIN_STEP:
        *yaw = 0.0f;
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        break;
    case GIMBAL_CALI_YAW_MAX_STEP:
        *yaw = GIMBAL_CALI_MOTOR_SET;
        *pitch = 0.0f;
        break;
    case GIMBAL_CALI_YAW_MIN_STEP:
        *yaw = -GIMBAL_CALI_MOTOR_SET;
        *pitch = 0.0f;
        break;
    default:
        *yaw = 0.0f;
        *pitch = 0.0f;
        break;
    }
}

/**
  * @brief          绝对角模式云台角度增量生成
  * @retval         none
  */
void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    gimbal_read_manual_input(yaw, pitch, control);
}

/**
  * @brief          绝对角模式云台角度增量生成
  * @retval         none
  */
void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    gimbal_read_manual_input(yaw, pitch, control);
}

/**
  * @brief          无力模式云台角度增量清零
  * @retval         none
  */
void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    (void)control;

    if (yaw == 0 || pitch == 0)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          自旋模式云台角度增量生成
  * @retval         none
  */
void gimbal_spin_control(float *yaw, float *pitch, gimbal_control_t *control)
{
    float manual_yaw = 0.0f;
    float manual_pitch = 0.0f;

    if (yaw == 0 || pitch == 0 || control == 0)
    {
        return;
    }

    gimbal_read_manual_input(&manual_yaw, &manual_pitch, control);
    *yaw = GIMBAL_SPIN_SPEED + manual_yaw;
    *pitch = manual_pitch;
}
