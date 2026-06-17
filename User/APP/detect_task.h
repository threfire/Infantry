/**
  * @file       detect_task.h
  * @brief      设备在线检测接口声明
  * @note       定义 TOE 设备编号、离线状态结构体和检测接口。
  */
#ifndef DETECT_TASK_H
#define DETECT_TASK_H

#include "struct_typedef.h"

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10
#define DBUS_RX_ACTIVE_HOLD_TIME 100U

enum errorList
{
    DBUS_TOE = 0,
    CHASSIS_MOTOR1_TOE,
    CHASSIS_MOTOR2_TOE,
    CHASSIS_MOTOR3_TOE,
    CHASSIS_MOTOR4_TOE,
    CHASSIS_MOTOR5_TOE,
    CHASSIS_MOTOR6_TOE,
    CHASSIS_MOTOR7_TOE,
    CHASSIS_MOTOR8_TOE,
    PLUCK_MOTOR_TOE,        // 拨弹 DJI 电机在线检测
    FRIC1_MOTOR_TOE,        // 摩擦轮 1 电机在线检测
    FRIC2_MOTOR_TOE,        // 摩擦轮 2 电机在线检测
    FRIC3_MOTOR_TOE,        // 摩擦轮 3 电机在线检测
    YAW_GIMBAL_MOTOR_TOE,   // yaw MIT 电机在线检测
    PITCH_GIMBAL_MOTOR_TOE, // pitch MIT 电机在线检测
    BOARD_GYRO_TOE,
    BOARD_ACCEL_TOE,
    BOARD_MAG_TOE,
    REFEREE_TOE,
    RM_IMU_TOE,
    OLED_TOE,

    ERROR_LIST_LENGHT,
};

typedef __packed struct
{
    uint32_t new_time;
    uint32_t last_time;
    uint32_t lost_time;
    uint32_t work_time;
    uint16_t set_offline_time : 12;
    uint16_t set_online_time : 12;
    uint8_t enable : 1;
    uint8_t priority : 4;
    uint8_t error_exist : 1;
    uint8_t is_lost : 1;
    uint8_t data_is_error : 1;
    fp32 frequency;
} error_t;

void detect_task(void *pvParameters);
bool_t toe_is_error(uint8_t err);
void detect_hook(uint8_t toe);
const error_t *get_error_list_point(void);

#endif
