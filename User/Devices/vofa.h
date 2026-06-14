#ifndef __VOFA_H__
#define __VOFA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "struct_typedef.h"

#define VOFA_CH_COUNT 6U
#define VOFA_CHASSIS_MOTOR_COUNT 4U
#define VOFA_ENABLE_CSV_TEXT 0

typedef struct
{
    float fdata[VOFA_CH_COUNT];
    uint8_t tail[4];
} VOFA_JustFloatFrame_t;

void VOFA_ServiceSend(void);
void VOFA_SendChassisMotorMeasure(uint8_t motor_idx);
void VOFA_SendChassisPowerDebug(uint8_t motor_idx);
void VOFA_SendChassisPowerCurrentDebug(void);
void VOFA_SendChassisSpeedAccel(void);
void VOFA_SendChassisRealSpeedCurrent(void);
void VOFA_SendChassisAnglePidDebug(void);
void VOFA_SendChassisTranslatePairDebug(void);
void VOFA_SendChassisMotionDebug(void);
void VOFA_SendGimbalFric(void);
void VOFA_SendGimbalYaw(void);
void VOFA_SendGimbalPitch(void);
void VOFA_SendGimbalYawPitchHalf(void);
void VOFA_SendGimbalStrum(void);


#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H__ */
