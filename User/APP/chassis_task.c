/**
  * @file       chassis_task.c
  * @brief      底盘任务入口与弱接口
  * @note       串联底盘模式、反馈、控制、发送的周期任务链路。
  */
#include "chassis_task.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

chassis_move_t chassis_move;

/**
  * @brief          底盘任务入口
  * @retval         none
  */
void chassis_task(void *pvParameters)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_move);

	while (1)
	{
		chassis_set_mode(&chassis_move);
		chassis_mode_change_control_transit(&chassis_move);
		chassis_feedback_update(&chassis_move);
		chassis_set_contorl(&chassis_move);
		chassis_control_loop(&chassis_move);
		chassis_send_cmd(&chassis_move);

		osDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

/**
  * @brief          底盘初始化弱接口
  * @retval         none
  */
__weak void chassis_init(chassis_move_t *chassis_move_init)
{
	(void)chassis_move_init;
}

/**
  * @brief          底盘模式设置弱接口
  * @retval         none
  */
__weak void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	(void)chassis_move_mode;
}

/**
  * @brief          底盘模式切换过渡弱接口
  * @retval         none
  */
__weak void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	(void)chassis_move_transit;
}

/**
  * @brief          底盘反馈更新弱接口
  * @retval         none
  */
__weak void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	(void)chassis_move_update;
}

/**
  * @brief          底盘控制量设置弱接口
  * @retval         none
  */
__weak void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	(void)chassis_move_control;
}

/**
  * @brief          底盘闭环控制弱接口
  * @retval         none
  */
__weak void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	(void)chassis_move_control_loop;
}

/**
  * @brief          底盘电机命令发送弱接口
  * @retval         none
  */
__weak void chassis_send_cmd(chassis_move_t *chassis_move_send)
{
	(void)chassis_move_send;
}
