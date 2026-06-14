/**
  * @file       usb_task.c
  * @brief      USB 通信任务占位接口
  * @note       预留 USB 更新、循环和发送链路。
  */
#include "usb_task.h"
#include "cmsis_os.h"

void usb_update();
void usb_loop();
void usb_send();
/**
  * @brief          USB 任务入口
  * @note           预留 USB 更新、计算和发送三个阶段。
  * @retval         none
  */
void usb_task(void)
{
	osDelay(USB_INIT_TIME);
	//已经初始化到main.c里面了
	while(1){
		//更新信息
		usb_update();
		//计算
		usb_loop();
		//发送消息
		usb_send();
		
	}
	
	
}
