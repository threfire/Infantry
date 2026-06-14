// Single-header core platform shim
#pragma once

extern "C" {
#include "../../core/platform.h"
}

#ifdef HOST_PLATFORM_SHIM_IMPL
#if defined(_WIN32)
#include <winsock2.h>
#include <windows.h>
#else
#include <time.h>
#endif
#include <stdint.h>

extern "C" uint32_t platform_get_tick_ms(void) {
#if defined(_WIN32)
    return (uint32_t)GetTickCount();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
#endif
}

extern "C" void platform_delay_ms(uint32_t ms) {
#if defined(_WIN32)
    Sleep(ms);
#else
    struct timespec req;
    req.tv_sec = ms / 1000u;
    req.tv_nsec = (long)((ms % 1000u) * 1000000ul);
    nanosleep(&req, nullptr);
#endif
}

extern "C" void* platform_memcpy(void *dst, const void *src, uint32_t n) {
    unsigned char *d = (unsigned char*)dst;
    const unsigned char *s = (const unsigned char*)src;
    while (n--) *d++ = *s++;
    return dst;
}

extern "C" void* platform_memset(void *s, int c, uint32_t n) {
    unsigned char *p = (unsigned char*)s;
    unsigned char v = (unsigned char)c;
    while (n--) *p++ = v;
    return s;
}

extern "C" void* platform_memmove(void *dst, const void *src, uint32_t n) {
    unsigned char *d = (unsigned char*)dst;
    const unsigned char *s = (const unsigned char*)src;
    if (d < s) { while (n--) *d++ = *s++; }
    else if (d > s) { d += n; s += n; while (n--) *--d = *--s; }
    return dst;
}

extern "C" int platform_memcmp(const void *s1, const void *s2, uint32_t n) {
    const unsigned char *a = (const unsigned char*)s1;
    const unsigned char *b = (const unsigned char*)s2;
    while (n--) { if (*a != *b) return (*a < *b) ? -1 : 1; a++; b++; }
    return 0;
}

extern "C" uint32_t platform_strlen(const char *s) {
    uint32_t len = 0; while (s && *s++) len++; return len;
}

extern "C" char* platform_strcpy(char *dst, const char *src) {
    char *ret = dst; while (src && (*dst++ = *src++)) {} return ret;
}

extern "C" int platform_strcmp(const char *s1, const char *s2) {
    while (s1 && s2 && *s1 && (*s1 == *s2)) { s1++; s2++; }
    return *(const unsigned char*)s1 - *(const unsigned char*)s2;
}
#endif // HOST_PLATFORM_SHIM_IMPL
