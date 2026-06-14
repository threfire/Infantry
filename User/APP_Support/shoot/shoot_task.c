/**
  * @file       shoot_task.c
  * @brief      发射弱接口封装
  * @note       为具体发射实现提供 shoot_init 和 shoot_control_loop 弱接口。
  */
#include "shoot_task.h"

/**
  * @brief          发射模块初始化弱接口
  * @note           具体发射实现用同名函数覆盖该弱接口。
  * @retval         none
  */
__weak void shoot_init(void)
{
}

/**
  * @brief          发射模块周期控制弱接口
  * @note           具体发射实现用同名函数覆盖该弱接口。
  * @retval         none
  */
__weak void shoot_control_loop(void)
{
}

/**
  * @brief          发射任务初始化封装
  * @retval         none
  */
void shoot_task_init(void)
{
    shoot_init();
}

/**
  * @brief          发射任务周期封装
  * @retval         none
  */
void shoot_task_loop(void)
{
    shoot_control_loop();
}
