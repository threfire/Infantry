/**
  * @file       shoot_3508.h
  * @brief      3508 发射控制接口声明
  * @note       声明发射模块初始化和周期控制接口。
  */
#ifndef SHOOT_3508_H
#define SHOOT_3508_H

#include "shoot_task.h"

#define SHOOT_FRICTION_CMD_ID 0x200U
#define SHOOT_STRUM_CMD_ID 0x200U
#define SHOOT_FRIC_RPM_TO_MPS (2.0f * PI * SHOOT_FRIC_WHEEL_RADIUS_M / 60.0f)
#define SHOOT_FRIC_MA_PER_A 1000.0f
#define SHOOT_STRUM_ECD_TO_RAD (2.0f * PI / 8192.0f)

#ifndef SHOOT_STRUM_RELEASE_LOCK_SPEED_RPM
#define SHOOT_STRUM_RELEASE_LOCK_SPEED_RPM 30.0f
#endif

#ifndef SHOOT_STRUM_TARGET_LPF_ALPHA
#define SHOOT_STRUM_TARGET_LPF_ALPHA 0.01f
#endif

void shoot_init(void);
void shoot_control_loop(void);

#endif
