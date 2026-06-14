#ifndef CAMERA_CONFIG_H
#define CAMERA_CONFIG_H

#include <stdint.h>
#include "../../core/comm_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 相机通道的逻辑通道号与优先级 */
#ifndef CAM_CH_ID
#define CAM_CH_ID 1u
#endif

#ifndef CAM_PRIORITY
#define CAM_PRIORITY 2u
#endif

/* FIFO 诊断容量（仅用于存储最近事件，便于调试） */
#ifndef CAM_FIFO_CAP
#define CAM_FIFO_CAP 10u
#endif

/* 负载字段开关（默认与主机示例一致） */
#ifndef CAM_PAYLOAD_HAS_FRAME_ID
#define CAM_PAYLOAD_HAS_FRAME_ID 1
#endif
#ifndef CAM_PAYLOAD_HAS_TS_US
#define CAM_PAYLOAD_HAS_TS_US 1
#endif

/* 上行带宽开关：0=关闭相机触发上报，1=开启 */
#ifndef CAM_EVENT_PUBLISH_ENABLE
#define CAM_EVENT_PUBLISH_ENABLE 0
#endif

/* 将相机事件打包到缓冲区：格式 [sid][可选字段...]，返回长度（字节） */
static inline uint16_t camera_pack_event(uint8_t *buf, uint16_t sid,
                                         uint32_t frame_id, uint64_t ts_us)
{
    uint16_t pos = 0;
    comm_write_u16_le(&buf[pos], sid); pos += 2;
#if CAM_PAYLOAD_HAS_FRAME_ID
    comm_write_u32_le(&buf[pos], frame_id); pos += 4;
#endif
#if CAM_PAYLOAD_HAS_TS_US
    comm_write_u64_le(&buf[pos], ts_us); pos += 8;
#endif
    return pos;
}

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_CONFIG_H */
