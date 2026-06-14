/**
  * @file       Omni_chassis.h
  * @brief      十字全向底盘控制接口声明
  * @note       声明底盘弱接口覆盖函数。
  */
#ifndef OMNI_CHASSIS_H
#define OMNI_CHASSIS_H

#include "chassis_task.h"

#ifndef PID_USUAL
#define PID_USUAL PID_POSITION
#endif

#ifndef PID_calc
#define PID_calc PID_Calc
#endif

/**
  * @brief          初始化底盘控制结构体和控制器状态
  * @retval         none
  */
void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          根据行为层刷新底盘模式
  * @retval         none
  */
void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          执行底盘模式切换状态过渡
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

/**
  * @brief          更新底盘电机、正解车速和姿态反馈
  * @retval         none
  */
void chassis_feedback_update(chassis_move_t *chassis_move_update);

/**
  * @brief          计算底盘目标速度和规划速度
  * @retval         none
  */
void chassis_set_contorl(chassis_move_t *chassis_move_control);

/**
  * @brief          执行底盘前馈、速度环和电流限制控制
  * @retval         none
  */
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

/**
  * @brief          发送底盘电机电流命令
  * @retval         none
  */
void chassis_send_cmd(chassis_move_t *chassis_move_send);

#endif
