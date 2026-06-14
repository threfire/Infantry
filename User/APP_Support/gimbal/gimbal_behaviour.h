/**
  * @file       gimbal_behaviour.h
  * @brief      云台行为模式接口声明
  * @note       定义云台行为枚举和行为层控制接口。
  */
#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H

#include <stdbool.h>

#include "gimbal_task.h"

typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_INIT,
    GIMBAL_CALI,
    GIMBAL_ABSOLUTE_ANGLE,
    GIMBAL_RELATIVE_ANGLE,
    GIMBAL_MOTIONLESS,
    GIMBAL_SPIN,
} gimbal_behaviour_e;

extern volatile gimbal_behaviour_e gimbal_behaviour;

#ifndef GIMBAL_SPIN_KEYBOARD
#define GIMBAL_SPIN_KEYBOARD KEY_PRESSED_OFFSET_SHIFT
#endif

#ifndef GIMBAL_ZERO_KEYBOARD
#define GIMBAL_ZERO_KEYBOARD KEY_PRESSED_OFFSET_X
#endif

#ifndef GIMBAL_RELATIVE_KEYBOARD
#define GIMBAL_RELATIVE_KEYBOARD KEY_PRESSED_OFFSET_C
#endif

#ifndef GIMBAL_SPIN_SPEED
#define GIMBAL_SPIN_SPEED 0.0f
#endif

/**
  * @brief          设置云台行为模式对应的电机控制模式
  * @retval         none
  */
void gimbal_behaviour_mode_set(gimbal_control_t *control);

/**
  * @brief          根据云台行为模式生成两轴角度增量
  * @retval         none
  */
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *control);

/**
  * @brief          输出云台对底盘停止的联动请求
  * @retval         true 表示请求底盘停止
  */
bool gimbal_cmd_to_chassis_stop(void);

/**
  * @brief          输出云台对射击停止的联动请求
  * @retval         true 表示请求射击停止
  */
bool gimbal_cmd_to_shoot_stop(void);

/**
  * @brief          更新云台行为状态机
  * @retval         none
  */
void gimbal_behavour_set(gimbal_control_t *control);

/**
  * @brief          无力模式云台角度增量清零
  * @retval         none
  */
void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *control);

/**
  * @brief          初始化模式云台回中控制量生成
  * @retval         none
  */
void gimbal_init_control(float *yaw, float *pitch, gimbal_control_t *control);

/**
  * @brief          云台校准模式控制量生成
  * @retval         none
  */
void gimbal_cali_control(float *yaw, float *pitch, gimbal_control_t *control);

/**
  * @brief          绝对角模式云台角度增量生成
  * @retval         none
  */
void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *control);

/**
  * @brief          相对角模式云台角度增量生成
  * @retval         none
  */
void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *control);

/**
  * @brief          静止模式云台角度增量清零
  * @retval         none
  */
void gimbal_motionless_control(float *yaw, float *pitch, gimbal_control_t *control);

/**
  * @brief          自旋模式云台角度增量生成
  * @retval         none
  */
void gimbal_spin_control(float *yaw, float *pitch, gimbal_control_t *control);

#endif
