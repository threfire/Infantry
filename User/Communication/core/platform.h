#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief          MUX 解码
  * @details        将 uproto 消息解码为 MUX 头和有效载荷
  * @param[in]      none
  * @retval         ms tick 值
  */
uint32_t platform_get_tick_ms(void);

/**
  * @brief          延时
  * @details        等待指定时间
  * @param[in]      ms 延时 ms
  * @retval         none
  */
void platform_delay_ms(uint32_t ms);

/**
  * @brief          内存拷贝
  * @details        将 src 指向的内存区域复制到 dst 指向的内存区域
  * @param[in]      dst 目标内存地址
  * @param[in]      src 源内存地址
  * @param[in]      n 复制的字节数
  * @retval         dst 地址
  */
void* platform_memcpy(void *dst, const void *src, uint32_t n);

/**
  * @brief          内存设置
  * @details        将 s 指向的内存区域设置为 c
  * @param[in]      s 目标内存地址
  * @param[in]      c 设置的值
  * @param[in]      n 设置的字节数
  * @retval         s 地址
  */
void* platform_memset(void *s, int c, uint32_t n);

/**
  * @brief          内存移动
  * @details        将 src 指向的内存区域复制到 dst 指向的内存区域
  * @param[in]      dst 目标内存地址
  * @param[in]      src 源内存地址
  * @param[in]      n 复制的字节数
  * @retval         dst 地址
  */
void* platform_memmove(void *dst, const void *src, uint32_t n);

/**
  * @brief          内存比较
  * @details        将 s1 指向的内存区域与 s2 指向的内存区域进行比较
  * @param[in]      s1 内存地址1
  * @param[in]      s2 内存地址2
  * @param[in]      n 比较的字节数
  * @retval         0 相等
  */
int platform_memcmp(const void *s1, const void *s2, uint32_t n);

/**
  * @brief          字符串长度
  * @details        计算字符串的长度
  * @param[in]      s 字符串
  * @retval         字符串长度
  */
uint32_t platform_strlen(const char *s);

/**
  * @brief          字符串拷贝
  * @details        将 src 指向的字符串复制到 dst 指向的内存区域
  * @param[in]      dst 目标内存地址
  * @param[in]      src 源字符串
  * @retval         dst 地址
  */
char* platform_strcpy(char *dst, const char *src);

/**
  * @brief          字符串拼接
  * @details        将 src 指向的字符串拼接到 dst 指向的字符串后面
  * @param[in]      dst 目标字符串
  * @param[in]      src 源字符串
  * @retval         0 成功
  */
int platform_strcmp(const char *s1, const char *s2);

#ifdef __cplusplus
}
#endif

#endif // PLATFORM_H
