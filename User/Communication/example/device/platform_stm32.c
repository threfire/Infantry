// Minimal STM32 platform glue: time and libc-lite
/**
  * @file       platform_stm32.c
  * @brief      STM32 平台适配（时间与精简 libc）
  * @details    为 core/uproto/comm 提供所需的 platform_* 接口：
  *             - 时间相关：platform_get_tick_ms / platform_delay_ms
  *             - 内存/字符串：memcpy/memset/memmove/memcmp/strlen/strcpy/strcmp
  *             建议由 CubeMX 初始化 HAL 与 SysTick，确保 HAL_GetTick 可用。
  */
#include "../../core/platform.h"
#include "stm32h7xx_hal.h"

/**
  * @brief          毫秒节拍（自上电）
  * @retval         ms 计数
  */
uint32_t platform_get_tick_ms(void)
{
    return HAL_GetTick();
}

/**
  * @brief          毫秒级延时
  * @param[in]      ms：延时毫秒
  */
void platform_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
  * @brief          内存拷贝
  * @param[out]     dst：目标缓冲
  * @param[in]      src：源缓冲
  * @param[in]      n：字节数
  * @retval         dst
  */
void* platform_memcpy(void *dst, const void *src, uint32_t n)
{
    uint8_t *d = (uint8_t*)dst;
    const uint8_t *s = (const uint8_t*)src;
    for (uint32_t i = 0; i < n; i++) d[i] = s[i];
    return dst;
}

/**
  * @brief          内存置值
  * @param[out]     s：目标缓冲
  * @param[in]      c：置为的值
  * @param[in]      n：字节数
  * @retval         s
  */
void* platform_memset(void *s, int c, uint32_t n)
{
    uint8_t *p = (uint8_t*)s;
    uint8_t v = (uint8_t)c;
    for (uint32_t i = 0; i < n; i++) p[i] = v;
    return s;
}

/**
  * @brief          内存移动（支持重叠区域）
  * @param[out]     dst：目标缓冲
  * @param[in]      src：源缓冲
  * @param[in]      n：字节数
  * @retval         dst
  */
void* platform_memmove(void *dst, const void *src, uint32_t n)
{
    uint8_t *d = (uint8_t*)dst;
    const uint8_t *s = (const uint8_t*)src;
    if (d == s || n == 0) return dst;
    if (d < s) {
        for (uint32_t i = 0; i < n; i++) d[i] = s[i];
    } else {
        for (uint32_t i = n; i > 0; i--) d[i - 1] = s[i - 1];
    }
    return dst;
}

/**
  * @brief          内存比较
  * @param[in]      s1：缓冲1
  * @param[in]      s2：缓冲2
  * @param[in]      n：字节数
  * @retval         0 相等，负值/正值 表示 s1<s2 / s1>s2
  */
int platform_memcmp(const void *s1, const void *s2, uint32_t n)
{
    const uint8_t *a = (const uint8_t*)s1;
    const uint8_t *b = (const uint8_t*)s2;
    for (uint32_t i = 0; i < n; i++) {
        if (a[i] != b[i]) return (a[i] < b[i]) ? -1 : 1;
    }
    return 0;
}

/**
  * @brief          字符串长度
  * @param[in]      s：C 字符串
  * @retval         长度（不含结尾’\0’）
  */
uint32_t platform_strlen(const char *s)
{
    uint32_t i = 0;
    if (!s) return 0;
    while (s[i] != '\0') i++;
    return i;
}

/**
  * @brief          字符串拷贝
  * @param[out]     dst：目标缓冲
  * @param[in]      src：源字符串
  * @retval         dst
  */
char* platform_strcpy(char *dst, const char *src)
{
    uint32_t i = 0;
    if (!dst) return dst;
    if (!src) { dst[0] = '\0'; return dst; }
    do {
        dst[i] = src[i];
    } while (src[i++] != '\0');
    return dst;
}

/**
  * @brief          字符串比较
  * @param[in]      s1：字符串1
  * @param[in]      s2：字符串2
  * @retval         0 相等，负值/正值 表示 s1<s2 / s1>s2
  */
int platform_strcmp(const char *s1, const char *s2)
{
    uint32_t i = 0;
    if (s1 == s2) return 0;
    if (!s1) return -1;
    if (!s2) return 1;
    while (s1[i] != '\0' && s2[i] != '\0') {
        if (s1[i] != s2[i]) return (s1[i] < s2[i]) ? -1 : 1;
        i++;
    }
    if (s1[i] == s2[i]) return 0;
    return (s1[i] == '\0') ? -1 : 1;
}
