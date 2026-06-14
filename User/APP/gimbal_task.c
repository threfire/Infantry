/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c
  * @brief      gimbal task entry and weak mechanism hooks
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"
#include "gravity_comp.h"
#include "shoot_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

gimbal_control_t gimbal_control;

static osThreadId_t gimbalTaskHandle = NULL;

/**
  * @brief          云台任务入口
  * @retval         none
  */
static void gimbal_task(void *pvParameters);

/**
  * @brief          创建云台控制任务
  * @retval         none
  */
void GimbalTask_Init(void)
{
    static const osThreadAttr_t gimbalTask_attributes = {
        .name = "gimbalTask",
        .stack_size = 1024 * 4,
        .priority = (osPriority_t) osPriorityHigh,
    };
    gimbalTaskHandle = osThreadNew(gimbal_task, NULL, &gimbalTask_attributes);
    (void)gimbalTaskHandle;
}

/**
  * @brief          云台任务入口
  * @retval         none
  */
static void gimbal_task(void *pvParameters)
{
    TickType_t last_wake_time;

    (void)pvParameters;

    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    gimbal_init(&gimbal_control);
    shoot_init();
    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        gimbal_set_mode(&gimbal_control);
        gimbal_mode_change_control_transit(&gimbal_control);
        gimbal_feedback_update(&gimbal_control);
        gimbal_set_control(&gimbal_control);
        gimbal_control_loop(&gimbal_control);
        gravity_comp_execute(&gimbal_control);
        shoot_control_loop();
        gimbal_send_cmd(&gimbal_control);

        vTaskDelayUntil(&last_wake_time, GIMBAL_CONTROL_TIME);
    }
}

/**
  * @brief          获取 yaw 电机控制结构体指针
  * @retval         yaw 电机控制结构体只读指针
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          获取 pitch 电机控制结构体指针
  * @retval         pitch 电机控制结构体只读指针
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
  * @brief          云台初始化弱接口
  * @retval         none
  */
__weak void gimbal_init(gimbal_control_t *control)
{
    (void)control;
}

/**
  * @brief          云台模式设置弱接口
  * @retval         none
  */
__weak void gimbal_set_mode(gimbal_control_t *control)
{
    (void)control;
}

/**
  * @brief          云台反馈更新弱接口
  * @retval         none
  */
__weak void gimbal_feedback_update(gimbal_control_t *control)
{
    (void)control;
}

/**
  * @brief          云台模式切换过渡弱接口
  * @retval         none
  */
__weak void gimbal_mode_change_control_transit(gimbal_control_t *control)
{
    (void)control;
}

/**
  * @brief          云台控制量设置弱接口
  * @retval         none
  */
__weak void gimbal_set_control(gimbal_control_t *control)
{
    (void)control;
}

/**
  * @brief          云台闭环控制弱接口
  * @retval         none
  */
__weak void gimbal_control_loop(gimbal_control_t *control)
{
    (void)control;
}

/**
  * @brief          云台命令发送弱接口
  * @retval         none
  */
__weak void gimbal_send_cmd(gimbal_control_t *control)
{
    (void)control;
}
