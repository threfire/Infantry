/**
  * @file       light_task.c
  * @brief      灯效任务与状态显示
  * @note       根据底盘、云台、超级电容和遥控状态刷新 WS2812 灯效。
  */
#include "light_task.h"

#include <string.h>

#include "bsp_usart.h"
#include "auto_aim.h"
#include "chassis_behaviour.h"
#include "chassis_power_control.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"
#include "shoot_task.h"

/* 灯位状态定义：0=DBUS 在线，1=底盘电机在线，2=摩擦轮在线，3=自瞄在线，4~6=底盘行为，7=云台行为，8=超级电容，9=任务心跳。 */



/* 灯板串口帧格式：帧头 0xAA 0x55，帧尾 0x55 0xAA，中间是 10 路灯的 RGB 数据。 */
extern super_cap_mode_e super_cap_mode;

static light_control_t light_control = {
    .task_handle = NULL,
    .mode = LIGHT_MODE_AUTO,
};

static void light_render_auto(void);
static void light_update_online_status(void);
static bool light_chassis_motor_online(void);
static bool light_fric_motor_online(void);
static bool light_fric_one_motor_online(const shoot_task_motor_t *motor, uint32_t now);
static bool light_fric_motor_working(void);
static bool light_fric_motor_running(void);
static float light_fric_average_speed_rpm(void);
static float light_abs_float(float value);
static void light_render_online_status(void);
static void light_render_chassis_status(void);
static void light_render_gimbal_status(void);
static void light_render_shoot_heat_status(void);
static void light_render_heartbeat(uint8_t tick);
static void light_pack_frame(void);
static void light_send_frame(void);
static void light_fill(uint8_t first, uint8_t last, uint8_t r, uint8_t g, uint8_t b);


/**
  * @brief          创建灯光任务
  * @note           任务只负责状态显示和串口发送，不参与控制闭环。
  * @retval         none
  */
void LightTask_Init(void)
{
    static const osThreadAttr_t lightTask_attributes = {
        .name = "lightTask",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityLow,
    };
    light_control.task_handle = osThreadNew(light_task, NULL, &lightTask_attributes);
}


/**
  * @brief          灯光任务主循环
  * @param[in]      pvParameters: FreeRTOS 任务参数
  * @note           自动模式周期刷新状态灯，手动模式保持外部写入的灯值并周期发送。
  * @retval         none
  */
void light_task(void *pvParameters)
{
    (void)pvParameters;

    osDelay(LIGHT_TASK_INIT_TIME_MS);
    light_clear();
    light_send_frame();

    while (1)
    {
        if (light_control.mode == LIGHT_MODE_AUTO)
        {
            light_render_auto();
        }

        light_send_frame();
        osDelay(LIGHT_TASK_PERIOD_MS);
    }
}


/* 自动灯效入口：按在线状态、底盘、云台、电容和心跳刷新灯板。 */
static void light_render_auto(void)
{
    static uint8_t tick = 0U;

    light_update_online_status();
    light_clear();

    light_render_online_status();
    light_render_chassis_status();
    light_render_gimbal_status();
    light_render_shoot_heat_status();
    light_render_heartbeat(tick);

    tick++;
}


/* 更新灯光任务内部的在线状态缓存，渲染层只读取这个缓存。 */
static void light_update_online_status(void)
{
    const uint8_t auto_aim_online = *((volatile uint8_t *)&aim.online);
    const uint8_t auto_aim_flag = *((volatile uint8_t *)&aim.auto_aim_flag);

    light_control.online.dbus_online =
        (toe_is_error(DBUS_TOE) == 0U);
    light_control.online.chassis_motor_online =
        light_chassis_motor_online();
    light_control.online.fric_motor_online =
        light_fric_motor_online();
    light_control.online.fric_motor_working =
        light_fric_motor_working();
    light_control.online.fric_motor_running =
        light_fric_motor_running();
    light_control.online.auto_aim_online =
        (auto_aim_online != 0U);
    light_control.online.auto_aim_active =
        ((auto_aim_online != 0U) && (auto_aim_flag == AIM_ON));
}


/* 判断 8 个底盘驱动电机是否全部在线。 */
static bool light_chassis_motor_online(void)
{
    for (uint8_t toe = CHASSIS_MOTOR1_TOE; toe <= CHASSIS_MOTOR4_TOE; toe++)
    {
        if (toe_is_error(toe))
        {
            return false;
        }
    }

    return true;
}


/* 判断 3 个摩擦轮电机是否全部在线。 */
static bool light_fric_motor_online(void)
{
    const uint32_t now = HAL_GetTick();

    return light_fric_one_motor_online(&shoot_task_control.fric1, now) &&
           light_fric_one_motor_online(&shoot_task_control.fric2, now) &&
           light_fric_one_motor_online(&shoot_task_control.fric3, now);
}


/* 判断单个摩擦轮电机是否在线。 */
static bool light_fric_one_motor_online(const shoot_task_motor_t *motor, uint32_t now)
{
    if ((motor == NULL) || (motor->measure == NULL))
    {
        return false;
    }

    if (motor->measure->last_fdb_time == 0U)
    {
        return false;
    }

    return ((now - motor->measure->last_fdb_time) <= LIGHT_FRIC_FDB_TIMEOUT);
}


/* 判断摩擦轮使能后是否达到工作转速。 */
static bool light_fric_motor_working(void)
{
    if (!shoot_task_control.friction_enable)
    {
        return true;
    }

    return (light_fric_average_speed_rpm() >= LIGHT_FRIC_WORK_MIN_RPM);
}


/* 判断摩擦轮使能后是否达到运行转速。 */
static bool light_fric_motor_running(void)
{
    if (!shoot_task_control.friction_enable)
    {
        return false;
    }

    return (light_fric_average_speed_rpm() > LIGHT_FRIC_RUNNING_RPM);
}


/* 计算 3 路摩擦轮平均转速的绝对值，单位 rpm。 */
static float light_fric_average_speed_rpm(void)
{
    return (light_abs_float(shoot_task_control.fric1.speed_rpm) +
            light_abs_float(shoot_task_control.fric2.speed_rpm) +
            light_abs_float(shoot_task_control.fric3.speed_rpm)) /
           3.0f;
}


/* 返回浮点绝对值，供阈值判断使用。 */
static float light_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}


/* 刷新 0~3 号在线状态灯。 */
static void light_render_online_status(void)
{
    if (light_control.online.dbus_online)
    {
        light_set_pixel(LIGHT_DBUS_ONLINE_LED, 0U, LIGHT_MID, 0U);
    }
    else
    {
        light_set_pixel(LIGHT_DBUS_ONLINE_LED, LIGHT_HIGH, 0U, 0U);
    }

    if (light_control.online.chassis_motor_online)
    {
        light_set_pixel(LIGHT_CHASSIS_ONLINE_LED, 0U, LIGHT_MID, 0U);
    }
    else
    {
        light_set_pixel(LIGHT_CHASSIS_ONLINE_LED, LIGHT_HIGH, 0U, 0U);
    }

    if (light_control.online.fric_motor_online)
    {
        if (!light_control.online.fric_motor_working)
        {
            light_set_pixel(LIGHT_FRIC_ONLINE_LED, LIGHT_HIGH, LIGHT_HIGH, 0U);
        }
        else if (light_control.online.fric_motor_running)
        {
            light_set_pixel(LIGHT_FRIC_ONLINE_LED, 0U, LIGHT_MID, LIGHT_MID);
        }
        else
        {
            light_set_pixel(LIGHT_FRIC_ONLINE_LED, 0U, LIGHT_MID, 0U);
        }
    }
    else
    {
        light_set_pixel(LIGHT_FRIC_ONLINE_LED, LIGHT_HIGH, 0U, 0U);
    }

    if (light_control.online.auto_aim_active)
    {
        light_set_pixel(LIGHT_AUTO_AIM_ONLINE_LED, 0U, LIGHT_MID, 0U);
    }
    else if (light_control.online.auto_aim_online)
    {
        light_set_pixel(LIGHT_AUTO_AIM_ONLINE_LED, LIGHT_HIGH, LIGHT_HIGH, 0U);
    }
    else
    {
        light_set_pixel(LIGHT_AUTO_AIM_ONLINE_LED, LIGHT_HIGH, 0U, 0U);
    }
}


/* 刷新 4~6 号灯，显示底盘行为模式。 */
static void light_render_chassis_status(void)
{
    switch (chassis_behaviour_mode)
    {
        case CHASSIS_NO_MOVE:
            light_fill(LIGHT_CHASSIS_STATUS_FIRST_LED,
                       LIGHT_CHASSIS_STATUS_LAST_LED,
                       0U,
                       0U,
                       LIGHT_DIM);
            break;

        case CHASSIS_FOLLOW_GIMBAL_YAW:
            light_fill(LIGHT_CHASSIS_STATUS_FIRST_LED,
                       LIGHT_CHASSIS_STATUS_LAST_LED,
                       0U,
                       LIGHT_MID,
                       LIGHT_MID);
            break;

        case CHASSIS_SPIN:
            light_fill(LIGHT_CHASSIS_STATUS_FIRST_LED,
                       LIGHT_CHASSIS_STATUS_LAST_LED,
                       LIGHT_LOW,
                       0U,
                       LIGHT_MID);
            break;

        case CHASSIS_RETURN:
        default:
            light_fill(LIGHT_CHASSIS_STATUS_FIRST_LED,
                       LIGHT_CHASSIS_STATUS_LAST_LED,
                       LIGHT_MID,
                       LIGHT_MID,
                       0U);
            break;
    }
}


/* 刷新 7 号灯，显示云台行为模式。 */
static void light_render_gimbal_status(void)
{
    switch (gimbal_behaviour)
    {
        case GIMBAL_ZERO_FORCE:
        case GIMBAL_MOTIONLESS:
            light_set_pixel(LIGHT_GIMBAL_STATUS_LED, LIGHT_LOW, 0U, 0U);
            break;

        case GIMBAL_INIT:
        case GIMBAL_CALI:
            light_set_pixel(LIGHT_GIMBAL_STATUS_LED, LIGHT_MID, LIGHT_MID, 0U);
            break;

        case GIMBAL_SPIN:
            light_set_pixel(LIGHT_GIMBAL_STATUS_LED, LIGHT_LOW, 0U, LIGHT_MID);
            break;

        case GIMBAL_ABSOLUTE_ANGLE:
        case GIMBAL_RELATIVE_ANGLE:
        default:
            light_set_pixel(LIGHT_GIMBAL_STATUS_LED, 0U, LIGHT_MID, 0U);
            break;
    }
}


/* 刷新 8 号灯，按发射热量从绿到黄再到红渐变。 */
static void light_render_shoot_heat_status(void)
{
    uint16_t heat = shoot_task_control.heat;
    uint8_t red = 0U;
    uint8_t green = 0U;
    const uint16_t half_heat = SHOOT_HEAT_LIMIT / 2U;

    if (heat >= SHOOT_HEAT_LIMIT || shoot_task_control.heat_limit_active)
    {
        light_set_pixel(LIGHT_SHOOT_HEAT_LED, LIGHT_HIGH, 0U, 0U);
        return;
    }

    if (half_heat == 0U)
    {
        light_set_pixel(LIGHT_SHOOT_HEAT_LED, LIGHT_HIGH, 0U, 0U);
        return;
    }

    if (heat <= half_heat)
    {
        red = (uint8_t)((uint32_t)heat * LIGHT_HIGH / half_heat);
        green = LIGHT_HIGH;
    }
    else
    {
        red = LIGHT_HIGH;
        green = (uint8_t)((uint32_t)(SHOOT_HEAT_LIMIT - heat) * LIGHT_HIGH / half_heat);
    }

    light_set_pixel(LIGHT_SHOOT_HEAT_LED, red, green, 0U);
}


/* 刷新 9 号灯作为任务心跳。 */
static void light_render_heartbeat(uint8_t tick)
{
    const uint8_t phase = tick % LIGHT_HEARTBEAT_BREATH_TICKS;
    const uint8_t ramp = (phase < (LIGHT_HEARTBEAT_BREATH_TICKS / 2U)) ?
                         phase :
                         (uint8_t)(LIGHT_HEARTBEAT_BREATH_TICKS - 1U - phase);
    const uint8_t brightness =
        (uint8_t)((uint16_t)ramp * LIGHT_LOW / ((LIGHT_HEARTBEAT_BREATH_TICKS / 2U) - 1U));

    light_set_pixel(LIGHT_HEARTBEAT_LED, brightness, 0U, brightness);
}


/* 把 light_control.leds[] 打包成灯板串口帧。 */
static void light_pack_frame(void)
{
    uint8_t frame_index = 0U;

    light_control.frame[frame_index++] = LIGHT_FRAME_HEAD0;
    light_control.frame[frame_index++] = LIGHT_FRAME_HEAD1;

    for (uint8_t i = 0U; i < LIGHT_LED_COUNT; i++)
    {
        light_control.frame[frame_index++] = light_control.leds[i].r;
        light_control.frame[frame_index++] = light_control.leds[i].g;
        light_control.frame[frame_index++] = light_control.leds[i].b;
    }

    light_control.frame[frame_index++] = LIGHT_FRAME_TAIL0;
    light_control.frame[frame_index] = LIGHT_FRAME_TAIL1;
}


/* 发送当前灯效帧。 */
static void light_send_frame(void)
{
    light_pack_frame();
    USART8_Transmit(light_control.frame, (uint16_t)sizeof(light_control.frame));
}


/* 将指定区间的灯珠设置为同一 RGB 颜色。 */
static void light_fill(uint8_t first, uint8_t last, uint8_t r, uint8_t g, uint8_t b)
{
    if (first >= LIGHT_LED_COUNT)
    {
        return;
    }

    if (last >= LIGHT_LED_COUNT)
    {
        last = LIGHT_LED_COUNT - 1U;
    }

    for (uint8_t i = first; i <= last; i++)
    {
        light_control.leds[i].r = r;
        light_control.leds[i].g = g;
        light_control.leds[i].b = b;
    }
}

/* 切换到自动灯效模式。 */
void light_set_auto_mode(void)
{
    light_control.mode = LIGHT_MODE_AUTO;
}

/* 切换到手动调灯模式，自动刷新暂停。 */
void light_set_manual_mode(void)
{
    light_control.mode = LIGHT_MODE_MANUAL;
}


/* 手动设置整板颜色。 */
void light_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    light_set_manual_mode();
    light_fill(0U, LIGHT_LED_COUNT - 1U, r, g, b);
}


/* 设置单颗灯珠颜色。 */
void light_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= LIGHT_LED_COUNT)
    {
        return;
    }

    light_control.leds[index].r = r;
    light_control.leds[index].g = g;
    light_control.leds[index].b = b;
}

/* 清空所有灯珠颜色。 */
void light_clear(void)
{
    memset(light_control.leds, 0, sizeof(light_control.leds));
}

/* 立即按当前灯值刷新灯板。 */
void light_refresh_now(void)
{
    light_send_frame();
}
