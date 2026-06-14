/**
  ****************************(C) COPYRIGHT 2026****************************
  * @file       service_task.h
  * @brief      service task, used for some ordinary jobs
  ****************************(C) COPYRIGHT 2026****************************
  */

#ifndef SERVICE_TASK_H
#define SERVICE_TASK_H

#include <stdint.h>
#include <stdbool.h>

#ifndef SERVICE_TASK_INIT_TIME
#define SERVICE_TASK_INIT_TIME  50
#endif

#ifndef SERVICE_CONTROL_TIME
#define SERVICE_CONTROL_TIME    1
#endif

typedef struct
{
    uint32_t service_time;
} service_control_t;

extern service_control_t service_control;

/* 对外初始化接口：由 freertos.c 调用 */
void ServiceTask_Init(void);

#endif
