#include "bsp_tim24.h"
#include "tim.h"

static volatile uint32_t tim24_hi = 0;

void tim24_timebase_init(void) {
    tim24_hi = 0;
    __HAL_TIM_SET_COUNTER(&htim24, 0);
    __HAL_TIM_CLEAR_FLAG(&htim24, TIM_FLAG_UPDATE);
    (void)HAL_TIM_Base_Start_IT(&htim24);
}

void tim24_timebase_on_overflow(void) {
    tim24_hi++;
}

uint64_t tim24_timebase_now_us(void) {
    uint32_t hi;
    uint32_t lo;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    hi = tim24_hi;
    lo = __HAL_TIM_GET_COUNTER(&htim24);
    if (__HAL_TIM_GET_FLAG(&htim24, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim24, TIM_FLAG_UPDATE);
        hi++;
        tim24_hi = hi;
        lo = __HAL_TIM_GET_COUNTER(&htim24);
    }
    if (!primask) {
        __enable_irq();
    }

    return ((uint64_t)hi << 32) | lo;
}
