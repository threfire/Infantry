#ifndef BSP_TIM24_H
#define BSP_TIM24_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void tim24_timebase_init(void);
uint64_t tim24_timebase_now_us(void);
void tim24_timebase_on_overflow(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TIM24_H */
