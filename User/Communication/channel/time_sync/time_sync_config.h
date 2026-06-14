#ifndef TIME_SYNC_CONFIG_H
#define TIME_SYNC_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 时间同步通道的逻辑通道号与默认优先级 */
#ifndef TS_CH_ID
#define TS_CH_ID 3u
#endif

#ifndef TS_PRIORITY
#define TS_PRIORITY 1u
#endif

/* 周期性发送 REQ 的周期（毫秒）与角色（1=作为发起者） */
#ifndef TS_PERIOD_MS
#define TS_PERIOD_MS 1000u
#endif

#ifndef TS_INITIATOR
#define TS_INITIATOR 1u
#endif

/* 可接受的最大 RTT（用于筛选映射样本，0 表示不过滤） */
#ifndef TS_MAX_RTT_US
#define TS_MAX_RTT_US 0u
#endif

#ifdef __cplusplus
}
#endif

#endif /* TIME_SYNC_CONFIG_H */
