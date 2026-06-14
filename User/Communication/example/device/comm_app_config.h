/**
  * @file       comm_app_config.h
  * @brief      设备侧通信应用配置参数
  * @details    任务属性、通道 ID、GPIO 触发和主机控制注入参数集中在这里维护。
  */

#ifndef COMM_APP_CONFIG_H
#define COMM_APP_CONFIG_H

#include <stdint.h>
#include "remote_control.h"
#include "robot_param.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 任务配置 */
#ifndef COMM_APP_STACK
#define COMM_APP_STACK            640u
#endif

#ifndef COMM_APP_PRIO
#define COMM_APP_PRIO             osPriorityRealtime
#endif

#ifndef COMM_APP_LOOP_DELAY_MS
#define COMM_APP_LOOP_DELAY_MS    1u
#endif

#ifndef COMM_APP_USB_ENUM_TIMEOUT_MS
#define COMM_APP_USB_ENUM_TIMEOUT_MS 5000u
#endif

/* 自定义演示流通道配置 */
#ifndef COMM_APP_STREAM_CH_ID
#define COMM_APP_STREAM_CH_ID     2u
#endif

#ifndef COMM_APP_STREAM_SID_DATA
#define COMM_APP_STREAM_SID_DATA  0x0101u
#endif

/* 相机 GPIO 触发轮询配置 */
#ifndef CAM_TRIGGER_ENABLE
#define CAM_TRIGGER_ENABLE        0
#endif

#ifndef CAM_TRIGGER_ACTIVE_HIGH
#define CAM_TRIGGER_ACTIVE_HIGH   1
#endif

/* TFmini 上报配置 */
#ifndef TFMINI_ENABLE
#define TFMINI_ENABLE 0
#endif

#ifndef TFMINI_PUB_PERIOD_MS
#define TFMINI_PUB_PERIOD_MS 20u
#endif

/* 主机底盘/射击命令注入到遥控器的默认映射 */
#ifndef CHASSIS_FOLLOW_CHANNEL
#define CHASSIS_FOLLOW_CHANNEL 1
#endif

#ifndef SHOOT_ON_KEYBOARD
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Z
#endif

#ifndef SHOOT_OFF_KEYBOARD
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_X
#endif

#ifdef __cplusplus
}
#endif

#endif /* COMM_APP_CONFIG_H */
