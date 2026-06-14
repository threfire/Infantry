/**
  * @file       detect_task.c
  * @brief      设备在线检测任务
  * @note       通过 detect_hook 刷新反馈时间戳，并由 toe_is_error 提供在线状态。
  */
#include "detect_task.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "flash_log.h"
#include "remote_control.h"
#include "task.h"
#include "usart.h"

static error_t error_list[ERROR_LIST_LENGHT + 1];
static uint8_t detect_inited = 0U;
static uint8_t detect_error_last[ERROR_LIST_LENGHT];

static bool_t detect_dbus_rx_active(uint32_t now)
{
    static uint32_t last_dma_remaining = 0U;
    static uint32_t last_rx_time = 0U;
    static uint8_t dma_seen = 0U;
    uint32_t dma_remaining;

    if ((huart5.hdmarx == NULL) || (huart5.hdmarx->Instance == NULL))
    {
        return 0U;
    }

    dma_remaining = __HAL_DMA_GET_COUNTER(huart5.hdmarx);
    if (dma_seen == 0U)
    {
        last_dma_remaining = dma_remaining;
        dma_seen = 1U;
        last_rx_time = now;
        return 0U;
    }

    if (dma_remaining != last_dma_remaining)
    {
        last_dma_remaining = dma_remaining;
        last_rx_time = now;
    }

    return (bool_t)((now - last_rx_time) <= DBUS_RX_ACTIVE_HOLD_TIME);
}

static void detect_init(uint32_t time)
{
    static const uint16_t set_item[ERROR_LIST_LENGHT][3] =
    {
        {30, 0, 17},
        {10, 0, 12},
        {10, 0, 11},
        {10, 0, 10},
        {10, 0, 9},
        {2, 3, 16},
        {2, 3, 15},
        {2, 3, 14},
        {2, 3, 13},
        {10, 10, 8},
        {100, 100, 8},
        {100, 100, 8},
        {100, 100, 8},
        {2, 3, 4},
        {2, 3, 3},
        {2, 3, 7},
        {5, 5, 7},
        {40, 200, 7},
        {100, 100, 5},
        {10, 10, 7},
        {100, 100, 1},
    };

    for (uint8_t i = 0U; i < ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_item[i][0];
        error_list[i].set_online_time = set_item[i][1];
        error_list[i].priority = set_item[i][2];
        error_list[i].enable = 1U;
        error_list[i].error_exist = 1U;
        error_list[i].is_lost = 1U;
        error_list[i].data_is_error = 0U;
        error_list[i].frequency = 0.0f;
        error_list[i].new_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
        detect_error_last[i] = error_list[i].error_exist;
    }

    error_list[ERROR_LIST_LENGHT].enable = 0U;
    detect_inited = 1U;
}

/**
  * @brief          设备在线检测任务入口
  * @param[in]      pvParameters: FreeRTOS 任务参数
  * @note           周期检查每个 TOE 的最后反馈时间，超时后置为离线。
  * @retval         none
  */
void detect_task(void *pvParameters)
{
    (void)pvParameters;

    detect_init(xTaskGetTickCount());
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
        uint32_t now = xTaskGetTickCount();

        for (uint8_t i = 0U; i < ERROR_LIST_LENGHT; i++)
        {
            if (error_list[i].enable == 0U)
            {
                continue;
            }

            if (i == DBUS_TOE)
            {
                uint32_t dbus_time = rc_ctrl.last_fdb;

                if (remoter.sbus_recever_time > dbus_time)
                {
                    dbus_time = remoter.sbus_recever_time;
                }

                if (dbus_time != 0U)
                {
                    error_list[i].new_time = dbus_time;
                }

                if (detect_dbus_rx_active(now))
                {
                    error_list[i].new_time = now;
                }
            }

            if ((now - error_list[i].new_time) > error_list[i].set_offline_time)
            {
                error_list[i].is_lost = 1U;
                error_list[i].error_exist = 1U;
                error_list[i].lost_time = now;
            }
            else if ((now - error_list[i].work_time) >= error_list[i].set_online_time)
            {
                error_list[i].is_lost = 0U;
                error_list[i].error_exist = 0U;
            }

            if (detect_error_last[i] != error_list[i].error_exist)
            {
                const char *detail = (error_list[i].error_exist != 0U) ? "lost" : "recover";

                (void)flash_log_enqueue_error(FLASH_LOG_SOURCE_DETECT,
                                              (int16_t)i,
                                              detail);
                detect_error_last[i] = error_list[i].error_exist;
            }
        }

        vTaskDelay(DETECT_CONTROL_TIME);
    }
}

/**
  * @brief          查询设备离线/错误状态
  * @param[in]      err: TOE 设备编号
  * @retval         1: 离线或错误，0: 在线
  */
bool_t toe_is_error(uint8_t err)
{
    if (!detect_inited)
    {
        detect_init(xTaskGetTickCount());
    }

    if (err >= ERROR_LIST_LENGHT)
    {
        return 0U;
    }

    return (bool_t)(error_list[err].error_exist == 1U);
}

/**
  * @brief          刷新设备反馈时间戳
  * @param[in]      toe: TOE 设备编号
  * @note           接收到对应设备反馈帧时调用，用于在线检测门控。
  * @retval         none
  */
void detect_hook(uint8_t toe)
{
    if (!detect_inited)
    {
        detect_init(xTaskGetTickCount());
    }

    if (toe >= ERROR_LIST_LENGHT)
    {
        return;
    }

    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();
    error_list[toe].is_lost = 0U;
    error_list[toe].error_exist = 0U;

    if (error_list[toe].new_time > error_list[toe].last_time)
    {
        error_list[toe].frequency = configTICK_RATE_HZ /
                                    (fp32)(error_list[toe].new_time - error_list[toe].last_time);
    }
}

const error_t *get_error_list_point(void)
{
    return error_list;
}
