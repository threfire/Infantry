/**
  * @file       Safewarning.h
  * @brief      蜂鸣器与安全提示接口声明
  * @note       定义提示类型和蜂鸣器控制接口。
  */
#ifndef __SAFEWARNING_H__
#define __SAFEWARNING_H__

#include <stdint.h>

void ws2812_task(void);

#define NOTE_1K 1000
#define NOTE_2K 2000
#define NOTE_3K 3000
#define NOTE_4K 4000
#define NOTE_5K 5000

// 提示音类型枚举
typedef enum {
    BEEP_POWER_ON = 0,    // 开机提示音
    BEEP_SHORT_PRESS,      // 短按提示音
    BEEP_DOUBLE_PRESS,       // 连按提示音
    BEEP_ERROR,            // 错误提示音
    BEEP_SUCCESS           // 成功提示音
} BeepType_t;

typedef struct {
    BeepType_t current_type;
    uint16_t step;
    uint16_t counter;
    uint8_t is_playing;
    uint16_t frequency;
    uint16_t duration;
} Beep_State_t;

typedef struct {
    uint16_t freq;
    uint16_t duration;
} Note_t;

// 初始化蜂鸣器
void Beep_Init(void);

// 播放提示音
void Beep_Play(BeepType_t type);

// 停止蜂鸣器
void Beep_Stop(void);

// 检查蜂鸣器是否正在播放
uint8_t Beep_IsPlaying(void);

// 蜂鸣器任务（需要在主循环或定时器中调用）
void Beep_Task(void);

#endif
