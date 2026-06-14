/**
  * @file       auto_aim.c
  * @brief      自瞄误差缓存与视觉数据接口
  * @note       接收视觉 yaw/pitch 偏差并提供给云台控制读取。
  */
#include "auto_aim.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "robot_param.h"
#include "stm32h7xx_hal.h"
#include "task.h"

#include <stdbool.h>
#include <stddef.h>
#include <math.h>

auto_aim_t aim;

static auto_aim_error_t s_auto_aim_error = {0};

static float auto_aim_calc_pitch_compensation_rad(float horizontal_distance_m);
static void auto_aim_init(auto_aim_t *aim_obj);
static void auto_aim_set(auto_aim_t *aim_obj);
static void auto_aim_feedback_update(auto_aim_t *aim_obj);

static float auto_aim_calc_pitch_compensation_rad(float horizontal_distance_m)
{
    const float drop_ratio =
        (AUTO_AIM_BALLISTIC_DROP_K_MM_PER_M2 * horizontal_distance_m) /
        AUTO_AIM_MM_PER_M;
    const float discriminant = 1.0f - 4.0f * drop_ratio * drop_ratio;
    float tan_comp;

    if (horizontal_distance_m <= 0.0f || drop_ratio <= 0.0f ||
        discriminant <= 0.0f)
    {
        return 0.0f;
    }

    tan_comp = (1.0f - sqrtf(discriminant)) / (2.0f * drop_ratio);

    return -atanf(tan_comp);
}

static void auto_aim_clear_error(void)
{
    taskENTER_CRITICAL();
    s_auto_aim_error.yaw_err_rad = 0.0f;
    s_auto_aim_error.pitch_err_rad = 0.0f;
    taskEXIT_CRITICAL();
}

/**
  * @brief          获取视觉 yaw 误差
  * @retval         yaw 误差，单位 rad
  */
float auto_aim_get_yaw_err_rad(void)
{
    float err;

    taskENTER_CRITICAL();
    err = s_auto_aim_error.yaw_err_rad;
    taskEXIT_CRITICAL();

    return err;
}

/**
  * @brief          获取视觉 pitch 误差
  * @retval         pitch 误差，单位 rad
  */
float auto_aim_get_pitch_err_rad(void)
{
    float err;

    taskENTER_CRITICAL();
    err = s_auto_aim_error.pitch_err_rad;
    taskEXIT_CRITICAL();

    return err;
}

/**
  * @brief          获取自瞄可用状态
  * @retval         1: 自瞄开启且在线，0: 自瞄不可用
  */
uint8_t auto_aim_is_active(void)
{
    return (uint8_t)((aim.auto_aim_flag == AIM_ON) && (aim.online != 0U));
}

/**
  * @brief          清空视觉误差缓存
  * @retval         none
  */
void auto_aim_reset_delta_accum(void)
{
    auto_aim_clear_error();
}

/**
  * @brief          自瞄任务入口
  * @param[in]      pvParameters: FreeRTOS 任务参数
  * @retval         none
  */
void auto_aim_task(void *pvParameters)
{
    (void)pvParameters;

    osDelay(AIM_INIT_TIME);
    auto_aim_init(&aim);

    while (1)
    {
        auto_aim_set(&aim);
        auto_aim_feedback_update(&aim);
        osDelay(AUTO_AIM_TIME);
    }
}

/**
  * @brief          写入视觉增量误差
  * @param[in]      dyaw_udeg: yaw 误差，单位 1e-6 deg
  * @param[in]      dpitch_udeg: pitch 误差，单位 1e-6 deg
  * @param[in]      status: 视觉状态字
  * @param[in]      ts_us: 视觉时间戳，单位 us
  * @retval         none
  */
void auto_aim_apply_delta_udeg(int32_t dyaw_udeg,
                               int32_t dpitch_udeg,
                               uint16_t status,
                               uint64_t ts_us)
{
    const bool no_aim_data = (dyaw_udeg == 0) && (dpitch_udeg == 0);
    const float yaw_err_rad =
        no_aim_data ? 0.0f : ((float)dyaw_udeg * AUTO_AIM_UDEG_TO_RAD);
    const float pitch_err_rad =
        no_aim_data ? 0.0f :
        (((float)dpitch_udeg * AUTO_AIM_UDEG_TO_RAD) +
         auto_aim_calc_pitch_compensation_rad(AUTO_AIM_BALLISTIC_DISTANCE_M));

    taskENTER_CRITICAL();
    s_auto_aim_error.yaw_err_rad = yaw_err_rad;
    s_auto_aim_error.pitch_err_rad = pitch_err_rad;
    aim.delta_yaw_udeg = dyaw_udeg;
    aim.delta_pitch_udeg = dpitch_udeg;
    aim.status = status;
    aim.ts_us = ts_us;
    taskEXIT_CRITICAL();

    aim.online = 1U;
    aim.last_fdb = HAL_GetTick();
}

static void auto_aim_init(auto_aim_t *aim_obj)
{
    if (aim_obj == NULL)
    {
        return;
    }

    aim_obj->online = 1U;
    aim_obj->auto_aim_flag = (AUTO_AIM_SOFT_ENABLE != 0) ? AIM_ON : AIM_OFF;
    aim_obj->last_fdb = 0U;
    aim_obj->delta_yaw_udeg = 0;
    aim_obj->delta_pitch_udeg = 0;
    aim_obj->status = 0U;
    aim_obj->ts_us = 0ULL;
    aim_obj->aim_rc = get_remote_control_point();

    auto_aim_clear_error();
}

static void auto_aim_set(auto_aim_t *aim_obj)
{
    static bool last_press_r = false;
    bool press_r;

    if (aim_obj == NULL)
    {
        return;
    }

#if ROBOT_MODE == release
    if (HAL_GetTick() - aim_obj->last_fdb > AUTO_AIM_TIMEOUT)
    {
        aim_obj->online = 0U;
        aim_obj->auto_aim_flag = AIM_OFF;
        auto_aim_clear_error();
        return;
    }
#endif

    press_r = (aim_obj->aim_rc != NULL) &&
              ((aim_obj->aim_rc->key.v & KEY_PRESSED_OFFSET_R) != 0U);

    if (press_r && !last_press_r)
    {
        aim_obj->auto_aim_flag =
            (aim_obj->auto_aim_flag == AIM_OFF) ? AIM_ON : AIM_OFF;
    }
    last_press_r = press_r;
}

static void auto_aim_feedback_update(auto_aim_t *aim_obj)
{
    if (aim_obj == NULL)
    {
        return;
    }

    if (aim_obj->auto_aim_flag == AIM_OFF)
    {
        auto_aim_clear_error();
    }
}
