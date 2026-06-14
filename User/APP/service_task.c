/**
  ****************************(C) COPYRIGHT 2026****************************
  * @file       service_task.c
  * @brief      service task, used for some ordinary jobs
  ****************************(C) COPYRIGHT 2026****************************
  */

#include "service_task.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "safewarning.h"
#include "hwt_imu.h"
#include "vofa.h"
#include "flash_log.h"
#include "task.h"
service_control_t service_control;

/**/
static osThreadId_t serviceTaskHandle = NULL;
static void service_task(void *pvParameters);
void ServiceTask_Init(void)
{
    static const osThreadAttr_t serviceTask_attributes = {
        .name = "serviceTask",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityLow,
    };
    serviceTaskHandle = osThreadNew(service_task, NULL, &serviceTask_attributes);
    (void)serviceTaskHandle;
}
/**/


/**
  * @brief          模块私有任务主流程
  * @param[in]      none
  * @retval         none
  */
static void service_task(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(SERVICE_TASK_INIT_TIME);
		Beep_Init();
		hwt_imu_init();
		flash_log_init();
		Beep_Play(BEEP_POWER_ON);
    service_control.service_time = 0;

    while (1)
    {
			
      service_control.service_time += SERVICE_CONTROL_TIME;
			ws2812_task();
			Beep_Task();
			VOFA_ServiceSend();
			flash_log_service();

			vTaskDelay(SERVICE_CONTROL_TIME);
    }
}
