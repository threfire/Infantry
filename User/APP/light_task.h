/**
  * @file       light_task.h
  * @brief      灯效任务接口声明
  * @note       提供灯效任务初始化、手动设置和立即刷新接口。
  */
#ifndef LIGHT_TASK_H
#define LIGHT_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os.h"

#define LIGHT_LED_COUNT 10U
#define LIGHT_RGB_BYTES (LIGHT_LED_COUNT * 3U)
#define LIGHT_UART_FRAME_BYTES (2U + LIGHT_RGB_BYTES + 2U)
#define LIGHT_TASK_INIT_TIME_MS 700U
#define LIGHT_TASK_PERIOD_MS 50U
#define LIGHT_FRAME_HEAD0 0xAAU
#define LIGHT_FRAME_HEAD1 0x55U
#define LIGHT_FRAME_TAIL0 0x55U
#define LIGHT_FRAME_TAIL1 0xAAU

#define LIGHT_LOW 4U
#define LIGHT_DIM LIGHT_LOW
#define LIGHT_MID LIGHT_LOW
#define LIGHT_HIGH LIGHT_LOW
#define LIGHT_HEARTBEAT_BREATH_TICKS 128U
#define LIGHT_FRIC_WORK_MIN_RPM 100.0f
#define LIGHT_FRIC_RUNNING_RPM 200.0f
#define LIGHT_FRIC_FDB_TIMEOUT 300U

#define LIGHT_DBUS_ONLINE_LED 0U
#define LIGHT_CHASSIS_ONLINE_LED 1U
#define LIGHT_FRIC_ONLINE_LED 2U
#define LIGHT_AUTO_AIM_ONLINE_LED 3U
#define LIGHT_CHASSIS_STATUS_FIRST_LED 4U
#define LIGHT_CHASSIS_STATUS_LAST_LED 6U
#define LIGHT_GIMBAL_STATUS_LED 7U
#define LIGHT_CAP_STATUS_LED 8U
#define LIGHT_SHOOT_HEAT_LED LIGHT_CAP_STATUS_LED
#define LIGHT_HEARTBEAT_LED 9U

typedef enum
{
    LIGHT_MODE_AUTO = 0,
    LIGHT_MODE_MANUAL,
} light_mode_t;

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} light_rgb_t;

typedef struct
{
    bool dbus_online;
    bool chassis_motor_online;
    bool fric_motor_online;
    bool fric_motor_working;
    bool fric_motor_running;
    bool auto_aim_online;
    bool auto_aim_active;
} light_online_t;

typedef struct
{
    osThreadId_t task_handle;
    volatile light_mode_t mode;
    light_rgb_t leds[LIGHT_LED_COUNT];
    uint8_t frame[LIGHT_UART_FRAME_BYTES];
    light_online_t online;
} light_control_t;

void LightTask_Init(void);
void light_task(void *pvParameters);

void light_set_auto_mode(void);
void light_set_manual_mode(void);
void light_set_all(uint8_t r, uint8_t g, uint8_t b);
void light_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void light_clear(void);
void light_refresh_now(void);

#endif
