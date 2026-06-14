/**
  * @file       flash_log.c
  * @brief      外部 Flash 错误日志后台落盘实现
  * @note       错误触发时只入 RAM 队列，外部 Flash 擦写由 service_task 低优先级调度。
  */
#include "flash_log.h"

#if (FLASH_LOG_ENABLE != 0U)

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

static flash_log_ctrl_t flash_log_control;

/**
  * @brief          计算 CRC16 校验值
  * @param[in]      data: 数据起始地址
  * @param[in]      len: 数据长度，单位 byte
  * @retval         CRC16 校验值
  */
static uint16_t flash_log_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

/**
  * @brief          计算日志记录校验值
  * @param[in]      record: 日志记录指针
  * @retval         CRC16 校验值
  */
static uint16_t flash_log_record_crc(const flash_log_record_t *record)
{
    flash_log_record_t temp = *record;

    temp.crc = 0U;
    return flash_log_crc16((const uint8_t *)&temp, (uint16_t)sizeof(temp));
}

/**
  * @brief          判断日志记录是否有效
  * @param[in]      record: 日志记录指针
  * @retval         1: 有效，0: 无效
  */
static bool_t flash_log_record_valid(const flash_log_record_t *record)
{
    if ((record->magic != FLASH_LOG_MAGIC) ||
        (record->text_len > FLASH_LOG_TEXT_LEN))
    {
        return 0U;
    }

    return (bool_t)(record->crc == flash_log_record_crc(record));
}

/**
  * @brief          判断 Flash 位置是否为空记录
  * @param[in]      record: 日志记录指针
  * @retval         1: 空记录，0: 非空记录
  */
static bool_t flash_log_record_empty(const flash_log_record_t *record)
{
    const uint8_t *data = (const uint8_t *)record;

    for (uint16_t i = 0U; i < (uint16_t)sizeof(*record); i++)
    {
        if (data[i] != 0xFFU)
        {
            return 0U;
        }
    }

    return 1U;
}

/**
  * @brief          扫描日志区并恢复下一次写入位置
  * @param[in]      none
  * @retval         none
  */
static void flash_log_scan(void)
{
    flash_log_record_t record;
    uint32_t addr = FLASH_LOG_REGION_BASE;
    uint32_t max_seq = 0U;
    uint32_t first_empty_addr = FLASH_LOG_REGION_END;

    for (uint32_t i = 0U; i < FLASH_LOG_RECORD_COUNT; i++)
    {
        if (OSPI_W25Qxx_ReadBuffer((uint8_t *)&record, addr, sizeof(record)) != OSPI_W25Qxx_OK)
        {
            break;
        }

        if (flash_log_record_empty(&record))
        {
            first_empty_addr = addr;
            break;
        }

        if (flash_log_record_valid(&record) && (record.seq >= max_seq))
        {
            max_seq = record.seq;
        }

        addr += FLASH_LOG_RECORD_SIZE;
    }

    flash_log_control.next_seq = max_seq + 1U;
    flash_log_control.next_addr = first_empty_addr;
    if (flash_log_control.next_addr >= FLASH_LOG_REGION_END)
    {
        flash_log_control.next_addr = FLASH_LOG_REGION_BASE;
    }
}

/**
  * @brief          填充一条 RAM 日志记录
  * @param[out]     record: 日志记录指针
  * @param[in]      seq: 日志单调递增序号
  * @param[in]      source: 日志来源，见 flash_log_source_e
  * @param[in]      code: 错误码或状态码
  * @param[in]      detail: 简短文本描述
  * @retval         none
  */
static void flash_log_fill_record(flash_log_record_t *record,
                                  uint32_t seq,
                                  uint16_t source,
                                  int16_t code,
                                  const char *detail)
{
    uint16_t text_len = 0U;

    memset(record, 0, sizeof(*record));
    record->magic = FLASH_LOG_MAGIC;
    record->seq = seq;
    record->tick_ms = HAL_GetTick();
    record->source = source;
    record->code = code;
    record->level = FLASH_LOG_LEVEL_ERROR;

    if (detail != NULL)
    {
        while ((text_len < FLASH_LOG_TEXT_LEN) && (detail[text_len] != '\0'))
        {
            record->text[text_len] = detail[text_len];
            text_len++;
        }
    }

    record->text_len = (uint8_t)text_len;
    record->crc = flash_log_record_crc(record);
}

/**
  * @brief          初始化外部 Flash 日志模块
  * @param[in]      none
  * @retval         none
  */
void flash_log_init(void)
{
    memset(&flash_log_control, 0, sizeof(flash_log_control));
    flash_log_scan();
    flash_log_control.inited = 1U;
}

/**
  * @brief          记录一条错误日志到 RAM 队列
  * @param[in]      source: 日志来源，见 flash_log_source_e
  * @param[in]      code: 错误码或状态码
  * @param[in]      detail: 简短文本描述
  * @retval         1: 入队成功，0: 入队失败
  */
bool_t flash_log_enqueue_error(uint16_t source, int16_t code, const char *detail)
{
    flash_log_record_t record;
    uint32_t seq;

    taskENTER_CRITICAL();
    if (flash_log_control.inited == 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    if (flash_log_control.count >= FLASH_LOG_QUEUE_DEPTH)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    seq = flash_log_control.next_seq++;
    taskEXIT_CRITICAL();

    flash_log_fill_record(&record, seq, source, code, detail);

    taskENTER_CRITICAL();
    if (flash_log_control.count >= FLASH_LOG_QUEUE_DEPTH)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    flash_log_control.queue[flash_log_control.head] = record;
    flash_log_control.head++;
    if (flash_log_control.head >= FLASH_LOG_QUEUE_DEPTH)
    {
        flash_log_control.head = 0U;
    }
    flash_log_control.count++;
    taskEXIT_CRITICAL();

    return 1U;
}

/**
  * @brief          后台写入一条 RAM 队列中的日志
  * @param[in]      none
  * @retval         none
  */
void flash_log_service(void)
{
    flash_log_record_t record;
    uint32_t write_addr;

    taskENTER_CRITICAL();
    if ((flash_log_control.inited == 0U) || (flash_log_control.count == 0U))
    {
        taskEXIT_CRITICAL();
        return;
    }
    record = flash_log_control.queue[flash_log_control.tail];
    write_addr = flash_log_control.next_addr;
    taskEXIT_CRITICAL();

    if ((write_addr % FLASH_LOG_SECTOR_SIZE) == 0U)
    {
        if (OSPI_W25Qxx_SectorErase(write_addr) != OSPI_W25Qxx_OK)
        {
            return;
        }
    }

    if (OSPI_W25Qxx_WriteBuffer((uint8_t *)&record,
                                write_addr,
                                sizeof(record)) != OSPI_W25Qxx_OK)
    {
        return;
    }

    taskENTER_CRITICAL();
    if (flash_log_control.count == 0U)
    {
        taskEXIT_CRITICAL();
        return;
    }

    flash_log_control.tail++;
    if (flash_log_control.tail >= FLASH_LOG_QUEUE_DEPTH)
    {
        flash_log_control.tail = 0U;
    }
    flash_log_control.count--;
    flash_log_control.next_addr = write_addr + FLASH_LOG_RECORD_SIZE;
    if (flash_log_control.next_addr >= FLASH_LOG_REGION_END)
    {
        flash_log_control.next_addr = FLASH_LOG_REGION_BASE;
    }
    taskEXIT_CRITICAL();
}

/**
  * @brief          通过串口按序输出 Flash 日志
  * @param[in]      huart: 串口句柄
  * @retval         none
  */
void flash_log_dump_to_uart(UART_HandleTypeDef *huart)
{
    flash_log_record_t record;
    char line[96];
    uint32_t last_seq = 0U;
    uint32_t best_seq;
    uint32_t best_addr;

    if ((flash_log_control.inited == 0U) || (huart == NULL))
    {
        return;
    }

    while (1)
    {
        best_seq = 0xFFFFFFFFUL;
        best_addr = FLASH_LOG_REGION_END;

        for (uint32_t i = 0U; i < FLASH_LOG_RECORD_COUNT; i++)
        {
            uint32_t addr = FLASH_LOG_REGION_BASE + (i * FLASH_LOG_RECORD_SIZE);

            if (OSPI_W25Qxx_ReadBuffer((uint8_t *)&record, addr, sizeof(record)) != OSPI_W25Qxx_OK)
            {
                return;
            }

            if (flash_log_record_valid(&record) &&
                (record.seq > last_seq) &&
                (record.seq < best_seq))
            {
                best_seq = record.seq;
                best_addr = addr;
            }
        }

        if (best_addr == FLASH_LOG_REGION_END)
        {
            break;
        }

        if (OSPI_W25Qxx_ReadBuffer((uint8_t *)&record, best_addr, sizeof(record)) != OSPI_W25Qxx_OK)
        {
            return;
        }

        record.text[FLASH_LOG_TEXT_LEN - 1U] = '\0';
        (void)snprintf(line, sizeof(line),
                       "[%lu] tick=%lu src=%u code=%d %s\r\n",
                       (unsigned long)record.seq,
                       (unsigned long)record.tick_ms,
                       record.source,
                       record.code,
                       record.text);
        (void)HAL_UART_Transmit(huart, (uint8_t *)line, (uint16_t)strlen(line), 100U);
        last_seq = best_seq;
    }
}

/**
  * @brief          擦除整个外部 Flash 日志区
  * @param[in]      none
  * @retval         none
  */
void flash_log_clear(void)
{
    if (flash_log_control.inited == 0U)
    {
        return;
    }

    for (uint32_t addr = FLASH_LOG_REGION_BASE;
         addr < FLASH_LOG_REGION_END;
         addr += FLASH_LOG_SECTOR_SIZE)
    {
        if (OSPI_W25Qxx_SectorErase(addr) != OSPI_W25Qxx_OK)
        {
            return;
        }
    }

    flash_log_control.head = 0U;
    flash_log_control.tail = 0U;
    flash_log_control.count = 0U;
    flash_log_control.next_addr = FLASH_LOG_REGION_BASE;
    flash_log_control.next_seq = 1U;
}

#else

/**
  * @brief          初始化外部 Flash 日志模块
  * @param[in]      none
  * @retval         none
  */
void flash_log_init(void)
{
}

/**
  * @brief          后台写入一条 RAM 队列中的日志
  * @param[in]      none
  * @retval         none
  */
void flash_log_service(void)
{
}

/**
  * @brief          记录一条错误日志到 RAM 队列
  * @param[in]      source: 日志来源，见 flash_log_source_e
  * @param[in]      code: 错误码或状态码
  * @param[in]      detail: 简短文本描述
  * @retval         1: 入队成功，0: 入队失败
  */
bool_t flash_log_enqueue_error(uint16_t source, int16_t code, const char *detail)
{
    (void)source;
    (void)code;
    (void)detail;
    return 0U;
}

/**
  * @brief          通过串口按序输出 Flash 日志
  * @param[in]      huart: 串口句柄
  * @retval         none
  */
void flash_log_dump_to_uart(UART_HandleTypeDef *huart)
{
    (void)huart;
}

/**
  * @brief          擦除整个外部 Flash 日志区
  * @param[in]      none
  * @retval         none
  */
void flash_log_clear(void)
{
}

#endif
