/**
  * @file       yaw_pitch_direct.h
  * @brief      yaw/pitch 直控云台接口与局部配置
  * @note       定义云台直控实现需要的默认宏、控制来源枚举和任务弱接口覆盖函数。
  */
#ifndef YAW_PITCH_DIRECT_H
#define YAW_PITCH_DIRECT_H

#include "gimbal_task.h"

#define YAW_PITCH_DIRECT_PI PI

#ifndef GIMBAL_AUTO_AIM_YAW_KP
#define GIMBAL_AUTO_AIM_YAW_KP 14.0f
#endif

#ifndef GIMBAL_AUTO_AIM_PITCH_KP
#define GIMBAL_AUTO_AIM_PITCH_KP 9.0f
#endif

#ifndef GIMBAL_AUTO_AIM_YAW_MAX_SPEED
#define GIMBAL_AUTO_AIM_YAW_MAX_SPEED (720.0f * YAW_PITCH_DIRECT_PI / 180.0f)
#endif

#ifndef GIMBAL_AUTO_AIM_PITCH_MAX_SPEED
#define GIMBAL_AUTO_AIM_PITCH_MAX_SPEED (720.0f * YAW_PITCH_DIRECT_PI / 180.0f)
#endif

#ifndef GIMBAL_AUTO_AIM_YAW_MAX_ACCEL
#define GIMBAL_AUTO_AIM_YAW_MAX_ACCEL (6000.0f * YAW_PITCH_DIRECT_PI / 180.0f)
#endif

#ifndef GIMBAL_AUTO_AIM_PITCH_MAX_ACCEL
#define GIMBAL_AUTO_AIM_PITCH_MAX_ACCEL (5100.0f * YAW_PITCH_DIRECT_PI / 180.0f)
#endif

#ifndef GIMBAL_PITCH_MIT_FDB_TIMEOUT
#define GIMBAL_PITCH_MIT_FDB_TIMEOUT 100U
#endif

typedef enum
{
    GIMBAL_CONTROL_SOURCE_RC = 0,
    GIMBAL_CONTROL_SOURCE_AUTO,
} gimbal_control_source_e;

/**
  * @brief          初始化云台控制结构体
  * @retval         none
  */
void gimbal_init(gimbal_control_t *control);

/**
  * @brief          设置云台控制模式
  * @retval         none
  */
void gimbal_set_mode(gimbal_control_t *control);

/**
  * @brief          更新云台反馈数据
  * @retval         none
  */
void gimbal_feedback_update(gimbal_control_t *control);

/**
  * @brief          处理云台模式切换过渡
  * @retval         none
  */
void gimbal_mode_change_control_transit(gimbal_control_t *control);

/**
  * @brief          生成云台目标控制量
  * @retval         none
  */
void gimbal_set_control(gimbal_control_t *control);

/**
  * @brief          执行云台闭环控制
  * @retval         none
  */
void gimbal_control_loop(gimbal_control_t *control);

/**
  * @brief          发送云台电机命令
  * @retval         none
  */
void gimbal_send_cmd(gimbal_control_t *control);

#endif
