#ifndef PROTOCOL_IDS_H
#define PROTOCOL_IDS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Base user msg type (use core default if included later) */
#ifndef UPROTO_MSG_USER_BASE
#define UPROTO_MSG_USER_BASE 0x10
#endif

/* Unified MUX carrier message type for both host/device */
#ifndef UPROTO_MSG_MUX
#define UPROTO_MSG_MUX (UPROTO_MSG_USER_BASE + 21)
#endif

/* Logical channel IDs (mirror device defaults) */
#ifndef CAM_CH_ID
#define CAM_CH_ID 1u
#endif
#ifndef TS_CH_ID
#define TS_CH_ID 3u
#endif
#ifndef GIMBAL_CH_ID
#define GIMBAL_CH_ID 4u
#endif
#ifndef TEMPLATE_CH_ID
#define TEMPLATE_CH_ID 5u
#endif

/* Common SIDs (dup for host convenience; device has own headers) */
#ifndef CAM_SID_EVENT
#define CAM_SID_EVENT 0x0001u
#endif
#ifndef CAM_SID_RESET
#define CAM_SID_RESET 0x0002u
#endif
#ifndef TS_SID_REQ
#define TS_SID_REQ   0x1001u
#endif
#ifndef TS_SID_RESP
#define TS_SID_RESP  0x1002u
#endif
#ifndef GIMBAL_SID_STATE
#define GIMBAL_SID_STATE 0x0201u
#endif
#ifndef GIMBAL_SID_DELTA
#define GIMBAL_SID_DELTA 0x0202u
#endif
#ifndef GIMBAL_SID_TFMINI
#define GIMBAL_SID_TFMINI 0x0203u
#endif

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_IDS_H */
