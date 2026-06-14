#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

#include <stdint.h>
#include "../../core/comm_utils.h"


#define gmb_le16 comm_write_u16_le
#define gmb_le32(p,v) comm_write_i32_le(p,v)
#define gmb_le64 comm_write_u64_le

#ifdef __cplusplus
extern "C" {
#endif

/* 逻辑通道号与默认优先级 */
#ifndef GIMBAL_CH_ID
#define GIMBAL_CH_ID 4u
#endif

#ifndef GIMBAL_PRIORITY
#define GIMBAL_PRIORITY 3u
#endif

/* 默认状态发布周期（毫秒） */
#ifndef GIMBAL_PUB_PERIOD_MS
#define GIMBAL_PUB_PERIOD_MS 10u
#endif

/* 状态负载字段开关（1 启用 / 0 关闭） */
#ifndef GIMBAL_STATE_HAS_ENCODERS
#define GIMBAL_STATE_HAS_ENCODERS 1
#endif
#ifndef GIMBAL_STATE_HAS_IMU
#define GIMBAL_STATE_HAS_IMU 1
#endif

/* 是否在收到主机 DELTA 命令后回 ACK（1=回执，0=不回，省带宽） */
#ifndef GIMBAL_DELTA_ACK_ENABLE
#define GIMBAL_DELTA_ACK_ENABLE 0
#endif

/* 上行带宽开关：TFmini 与裁判信息 */
#ifndef GIMBAL_TFMINI_PUBLISH_ENABLE
#define GIMBAL_TFMINI_PUBLISH_ENABLE 0
#endif

#ifndef GIMBAL_REFEREE_PUBLISH_ENABLE
#define GIMBAL_REFEREE_PUBLISH_ENABLE 1
#endif


/* 可选扩展钩子：定义为一个宏，用于在状态负载末尾追加自定义字节，并返回追加的长度（字节数）。
 * 示例：
 *   #define GIMBAL_STATE_PACK_EXT(p,enc_y,enc_p,yaw,pitch,roll,ts) \\
 *           ( gmb_le32((p), my_temp_cdeg), 4 )
 */
#ifndef GIMBAL_STATE_PACK_EXT
#define GIMBAL_STATE_PACK_EXT(p, enc_y, enc_p, yaw_u, pitch_u, roll_u, ts_u) (0)
#endif

/* 按照上述开关把云台状态打包到缓冲区，返回总长度（字节） */
static inline uint16_t gimbal_pack_state(uint8_t *buf, uint16_t sid,
                                         int32_t enc_yaw, int32_t enc_pitch,
                                         int32_t yaw_udeg, int32_t pitch_udeg, int32_t roll_udeg,
                                         uint64_t ts_us)
{
    uint16_t pos = 0;
    gmb_le16(&buf[pos], sid); pos += 2;
#if GIMBAL_STATE_HAS_ENCODERS
    gmb_le32(&buf[pos], enc_yaw);   pos += 4;
    gmb_le32(&buf[pos], enc_pitch); pos += 4;
#endif
#if GIMBAL_STATE_HAS_IMU
    gmb_le32(&buf[pos], yaw_udeg);   pos += 4;
    gmb_le32(&buf[pos], pitch_udeg); pos += 4;
    gmb_le32(&buf[pos], roll_udeg);  pos += 4;
#endif
    gmb_le64(&buf[pos], ts_us); pos += 8;
    /* Optional extension controlled by config */
    pos = (uint16_t)(pos + GIMBAL_STATE_PACK_EXT(&buf[pos], enc_yaw, enc_pitch, yaw_udeg, pitch_udeg, roll_udeg, ts_us));
    return pos;
}

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_CONFIG_H */
