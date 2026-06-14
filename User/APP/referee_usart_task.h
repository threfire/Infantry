/**
  * @file       referee_usart_task.h
  * @brief      裁判串口任务接口声明
  * @note       声明裁判串口任务初始化和任务入口。
  */
#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H

#include "main.h"
#include "robot_param.h"
#include "protocol.h"

extern uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

void RefereeUsartTask_Init(void);
void referee_usart_task(void *argument);

#endif
