/**
  * @file       flash_log.h
  * @brief      外部 Flash 错误日志接口
  * @note       错误触发时只写入 RAM 队列，Flash 写入由低优先级后台服务函数执行。
  */
#ifndef FLASH_LOG_H
#define FLASH_LOG_H

#include "main.h"
#include "robot_param.h"
#include "struct_typedef.h"

#if (FLASH_LOG_ENABLE != 0U)
#include "w25q64.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if (FLASH_LOG_ENABLE != 0U)

#define FLASH_LOG_MAGIC              0x464C4F47UL                  // 记录有效标识，ASCII 为 FLOG
#define FLASH_LOG_RECORD_SIZE        64U                           // 单条日志长度，单位 byte
#define FLASH_LOG_TEXT_LEN           44U                           // 单条日志文本长度，单位 byte
#define FLASH_LOG_SECTOR_SIZE        4096U                         // W25Q64 扇区大小，单位 byte
#define FLASH_LOG_REGION_SIZE        (64U * 1024U)                 // 日志区大小，单位 byte
#define FLASH_LOG_REGION_BASE        (W25Qxx_FlashSize - FLASH_LOG_REGION_SIZE) // 日志区起始地址
#define FLASH_LOG_REGION_END         W25Qxx_FlashSize              // 日志区结束地址
#define FLASH_LOG_RECORD_COUNT       (FLASH_LOG_REGION_SIZE / FLASH_LOG_RECORD_SIZE) // 日志区记录容量

#endif

typedef enum
{
    FLASH_LOG_LEVEL_INFO = 0U,   // 普通状态事件
    FLASH_LOG_LEVEL_WARN = 1U,   // 可恢复异常事件
    FLASH_LOG_LEVEL_ERROR = 2U,  // 错误事件
} flash_log_level_e;

typedef enum
{
    FLASH_LOG_SOURCE_SYSTEM = 0U, // 系统初始化和全局状态
    FLASH_LOG_SOURCE_DETECT = 1U, // 在线检测 TOE 状态
    FLASH_LOG_SOURCE_OSPI = 2U,   // 外部 Flash/OSPI 状态
} flash_log_source_e;

#if (FLASH_LOG_ENABLE != 0U)

typedef __packed struct
{
    uint32_t magic;                  // 记录有效标识
    uint32_t seq;                    // 单调递增序号
    uint32_t tick_ms;                // 记录时间，单位 ms
    uint16_t source;                 // 来源模块，见 flash_log_source_e
    int16_t code;                    // 错误码或状态码
    uint8_t level;                   // 日志等级，见 flash_log_level_e
    uint8_t text_len;                // 文本长度，单位 byte
    uint16_t crc;                    // 记录校验值
    char text[FLASH_LOG_TEXT_LEN];   // 简短文本
} flash_log_record_t;

typedef struct
{
    flash_log_record_t queue[FLASH_LOG_QUEUE_DEPTH]; // RAM 日志队列
    uint8_t head;                                    // 队列写入位置
    uint8_t tail;                                    // 队列读取位置
    uint8_t count;                                   // 队列已有记录数量
    uint8_t inited;                                  // 初始化完成标志
    uint32_t next_addr;                              // 下一条 Flash 写入地址
    uint32_t next_seq;                               // 下一条日志序号
} flash_log_ctrl_t;

#endif

/**
  * @brief          初始化外部 Flash 日志模块
  * @param[in]      none
  * @retval         none
  */
void flash_log_init(void);

/**
  * @brief          后台写入一条 RAM 队列中的日志
  * @param[in]      none
  * @retval         none
  */
void flash_log_service(void);

/**
  * @brief          记录一条错误日志到 RAM 队列
  * @param[in]      source: 日志来源，见 flash_log_source_e
  * @param[in]      code: 错误码或状态码
  * @param[in]      detail: 简短文本描述
  * @retval         1: 入队成功，0: 入队失败
  */
bool_t flash_log_enqueue_error(uint16_t source, int16_t code, const char *detail);

/**
  * @brief          通过串口按序输出 Flash 日志
  * @param[in]      huart: 串口句柄
  * @retval         none
  */
void flash_log_dump_to_uart(UART_HandleTypeDef *huart);

/**
  * @brief          擦除整个外部 Flash 日志区
  * @param[in]      none
  * @retval         none
  */
void flash_log_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_LOG_H */
