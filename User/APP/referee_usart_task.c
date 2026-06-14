/**
  * @file       referee_usart_task.c
  * @brief      裁判串口接收任务
  * @note       从串口 DMA 数据流中解析裁判系统帧并更新裁判数据。
  */
#include "referee_usart_task.h"

#include "string.h"
#include "cmsis_os.h"
#include "CRC8_CRC16.h"
#include "bsp_usart.h"
#include "fifo.h"
#include "referee.h"

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

static osThreadId_t refereeUsartTaskHandle = NULL;
static unpack_data_t referee_unpack_data = {0};

static void referee_unpack_fifo_data(void);

/**
  * @brief          创建裁判串口解析任务
  * @retval         none
  */
void RefereeUsartTask_Init(void)
{
    static const osThreadAttr_t refereeUsartTask_attributes = {
        .name = "refereeUsartTask",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };
    refereeUsartTaskHandle = osThreadNew(referee_usart_task, NULL, &refereeUsartTask_attributes);
    (void)refereeUsartTaskHandle;
}

/**
  * @brief          裁判串口任务入口
  * @param[in]      argument: FreeRTOS 任务参数
  * @retval         none
  */
void referee_usart_task(void *argument)
{
    (void)argument;

    init_referee_data();
    memset(&referee_unpack_data, 0, sizeof(referee_unpack_data));
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    referee_fifo_ready = 1;

    while (1)
    {
        referee_unpack_fifo_data();
        osDelay(10);
    }
}

static void referee_unpack_fifo_data(void)
{
    uint8_t byte = 0;
    uint16_t frame_len = 0;
    unpack_data_t *p_unpack = &referee_unpack_data;

    while (fifo_s_used(&referee_fifo))
    {
        byte = (uint8_t)fifo_s_get(&referee_fifo);

        switch (p_unpack->unpack_step)
        {
            case STEP_HEADER_SOF:
            {
                if (byte == HEADER_SOF)
                {
                    p_unpack->protocol_packet[p_unpack->index++] = byte;
                    p_unpack->unpack_step = STEP_LENGTH_LOW;
                }
                else
                {
                    p_unpack->index = 0;
                }
            } break;

            case STEP_LENGTH_LOW:
            {
                p_unpack->data_length = byte;
                p_unpack->protocol_packet[p_unpack->index++] = byte;
                p_unpack->unpack_step = STEP_LENGTH_HIGH;
            } break;

            case STEP_LENGTH_HIGH:
            {
                p_unpack->data_length |= ((uint16_t)byte << 8);
                p_unpack->protocol_packet[p_unpack->index++] = byte;

                if (p_unpack->data_length <= (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
                {
                    p_unpack->unpack_step = STEP_FRAME_SEQ;
                }
                else
                {
                    p_unpack->unpack_step = STEP_HEADER_SOF;
                    p_unpack->index = 0;
                }
            } break;

            case STEP_FRAME_SEQ:
            {
                p_unpack->protocol_packet[p_unpack->index++] = byte;
                p_unpack->unpack_step = STEP_HEADER_CRC8;
            } break;

            case STEP_HEADER_CRC8:
            {
                p_unpack->protocol_packet[p_unpack->index++] = byte;

                if (p_unpack->index == REF_PROTOCOL_HEADER_SIZE)
                {
                    if (verify_CRC8_check_sum(p_unpack->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
                    {
                        p_unpack->unpack_step = STEP_DATA_CRC16;
                    }
                    else
                    {
                        p_unpack->unpack_step = STEP_HEADER_SOF;
                        p_unpack->index = 0;
                    }
                }
            } break;

            case STEP_DATA_CRC16:
            {
                frame_len = REF_HEADER_CRC_CMDID_LEN + p_unpack->data_length;

                if (p_unpack->index < frame_len)
                {
                    p_unpack->protocol_packet[p_unpack->index++] = byte;
                }

                if (p_unpack->index >= frame_len)
                {
                    if (verify_CRC16_check_sum(p_unpack->protocol_packet, frame_len))
                    {
                        referee_handle_data(p_unpack->protocol_packet);
                    }

                    p_unpack->unpack_step = STEP_HEADER_SOF;
                    p_unpack->index = 0;
                }
            } break;

            default:
            {
                p_unpack->unpack_step = STEP_HEADER_SOF;
                p_unpack->index = 0;
            } break;
        }
    }
}
