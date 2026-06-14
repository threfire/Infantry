/**
  * @file       Safewarning.c
  * @brief      蜂鸣器与安全提示控制
  * @note       管理蜂鸣器提示节奏和 WS2812 安全提示输出。
  */
#include "safewarning.h"
#include "cmsis_os.h"
#include "ws2812.h"
#include "tim.h"

uint8_t r = 1;
uint8_t g = 1;
uint8_t b = 1;

/**
  * @brief          WS2812 测试灯效任务
  * @note           周期改变 RGB 值并刷新灯带。
  * @retval         none
  */
void ws2812_task(void)
{
    static uint16_t ws_cnt = 0;

    ws_cnt++;
    if (ws_cnt < 100)
    {
        return;
    }
    ws_cnt = 0;

    WS2812_Ctrl(r, g, b);
    r++;
    g += 5;
    b += 10;
    r++;
    g++;
    b++;
}

/**
  * @brief          蜂鸣器 PWM 测试输出
  * @retval         none
  */
void beep_test(void)
{
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	TIM12->CCR2 = 1800;
}

static Beep_State_t beep_state = {0};

static const Note_t power_on_melody[] = {
    {1000, 100},   // 滴
	{2000, 100},   // 滴
	{3000, 200},   // 滴）
	{1000, 50},   // 滴）
    {0, 0}
};

// 短按提示音：短促的"滴"（4kHz）
static const Note_t short_press_melody[] = {
    {NOTE_4K, 100},
    {0, 50},
    {0, 0}
};

// 连按提示音：低频长音（使用2kHz和4kHz交替，突出长按感）
static const Note_t long_press_melody[] = {
    {NOTE_2K, 300},
    {0, 100},
    {NOTE_4K, 200},
    {0, 100},
    {NOTE_4K, 400},
    {0, 0}
};

// 错误提示音：急促的下降音（从4kHz到1kHz）
static const Note_t error_melody[] = {
    {NOTE_4K, 100},
    {NOTE_3K, 100},
    {NOTE_2K, 100},
    {NOTE_1K, 200},
    {0, 0}
};

// 成功提示音：欢快的双音（4kHz与5kHz交替，5kHz略高于谐振点但仍在可接受范围）
static const Note_t success_melody[] = {
    {NOTE_4K, 100},
    {0, 50},
    {NOTE_4K, 100},
    {NOTE_5K, 150},
    {NOTE_4K, 200},
    {0, 0}
};

// 获取对应提示音的音符序列
static const Note_t* GetMelody(BeepType_t type) {
    switch(type) {
        case BEEP_POWER_ON:     return power_on_melody;
        case BEEP_SHORT_PRESS:  return short_press_melody;
        case BEEP_DOUBLE_PRESS:   return long_press_melody;
        case BEEP_ERROR:        return error_melody;
        case BEEP_SUCCESS:      return success_melody;
        default:                return NULL;
    }
}

// 设置PWM频率
static void SetBeepFrequency(uint16_t freq) {
    uint32_t tim_freq = 1000000;  // 定时器计数时钟为1MHz（PSC=239）
    uint32_t arr, ccr;
    
    if (freq == 0) {
        HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
        return;
    }
    
    arr = tim_freq / freq;
    ccr = arr / 2;  // 50%占空比
    
    __HAL_TIM_SET_AUTORELOAD(&htim12, arr - 1);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, ccr);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

/**
  * @brief          初始化蜂鸣器状态
  * @retval         none
  */
void Beep_Init(void) {
    beep_state.is_playing = 0;
    beep_state.step = 0;
    beep_state.counter = 0;
    Beep_Stop();
}

/**
  * @brief          播放指定提示音
  * @param[in]      type: 提示音类型
  * @retval         none
  */
void Beep_Play(BeepType_t type) {
    if (beep_state.is_playing) {
        Beep_Stop();
    }
    
    beep_state.current_type = type;
    beep_state.step = 0;
    beep_state.counter = 0;
    beep_state.is_playing = 1;
}

/**
  * @brief          停止蜂鸣器输出
  * @retval         none
  */
void Beep_Stop(void) {
    HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
    beep_state.is_playing = 0;
    beep_state.step = 0;
    beep_state.counter = 0;
}

/**
  * @brief          查询蜂鸣器播放状态
  * @retval         1: 正在播放，0: 空闲
  */
uint8_t Beep_IsPlaying(void) {
    return beep_state.is_playing;
}

/**
  * @brief          蜂鸣器周期任务
  * @note           每 1ms 调用一次，用于推进当前提示音音符序列。
  * @retval         none
  */
void Beep_Task(void) {
    if (!beep_state.is_playing) {
        return;
    }
    
    const Note_t* melody = GetMelody(beep_state.current_type);
    if (melody == NULL) {
        beep_state.is_playing = 0;
        return;
    }
    
    Note_t current_note = melody[beep_state.step];
    
    if (current_note.duration == 0 && current_note.freq == 0) {
        Beep_Stop();
        return;
    }
    
    beep_state.counter++;
    
    if (beep_state.counter == 1) {
        SetBeepFrequency(current_note.freq);
    }
    
    if (beep_state.counter >= current_note.duration) {
        beep_state.step++;
        beep_state.counter = 0;
    }
}
