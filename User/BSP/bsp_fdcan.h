#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__

#include "main.h"
#include "fdcan.h"
#include "stdbool.h"

#define hcan_t FDCAN_HandleTypeDef

typedef struct
{
    __packed struct
    {
        int id;
        int state;
        int p_int;
        int v_int;
        int t_int;
        int kp_int;
        int kd_int;
        float pos;
        float vel;
        float tor;
        float Kp;
        float Kd;
        float t_mos;
        float t_motor;
        uint32_t last_fdb_time;
    } fdb;
    __packed struct
    {
        float KP;
        float KD;
        float POS;
        float VEL;
        float TOR;
    } set;
    __packed struct
    {
        float P_min;
        float P_max;
        float V_min;
        float V_max;
        float KP_min;
        float KP_max;
        float KD_min;
        float KD_max;
        float T_min;
        float T_max;
    } param;
} MITMeasure_t;

typedef struct
{
    __IO bool rxFrameFlag;
} CAN_t;

extern __IO CAN_t can;

typedef enum
{
    CAN_ERROR_NONE          = 0x00,
    CAN_ERROR_WARNING       = 0x01,
    CAN_ERROR_PASSIVE       = 0x02,
    CAN_ERROR_BUS_OFF       = 0x04,
    CAN_ERROR_PROTOCOL_ARB  = 0x08,
    CAN_ERROR_PROTOCOL_DATA = 0x10,
    CAN_ERROR_STUFF         = 0x20,
    CAN_ERROR_FORM          = 0x40,
    CAN_ERROR_ACK           = 0x80,
    CAN_ERROR_CRC           = 0x100,
    CAN_ERROR_SEND          = 0x200,
} CAN_ErrorStatus;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    uint32_t last_fdb_time;
} motor_measure_t;

extern __IO CAN_ErrorStatus can_error_status;
extern MITMeasure_t MIT_MOTOR_MEASURE[4];
extern motor_measure_t DJI_MOTOR_MEASURE[8];
extern motor_measure_t CHASSIS_MOTOR_MEASURE[4];

void bsp_can_init(void);
void can1_filter_init(void);
void can2_filter_init(void);
void can3_filter_init(void);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcan1_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
uint8_t fdcan2_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
uint8_t fdcan3_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
void CAN_cmd_MIT(FDCAN_HandleTypeDef *hcan, uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
void CAN_cmd_CHAS_3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_CHAS_6020(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
void CAN_cmd_CHASSIS_ALL(int16_t motor205, int16_t motor206, int16_t motor207, int16_t motor208);
void CAN_cmd_SHOOT_ALL(int16_t motor201, int16_t motor202, int16_t motor203, int16_t motor204);
motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
void Motor_save_zero(FDCAN_HandleTypeDef *hcan, uint16_t id);
void Motor_ENABLE(FDCAN_HandleTypeDef *hcan, uint16_t id);
void Motor_MIT_MODE(FDCAN_HandleTypeDef *hcan, uint16_t id);
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x, float x_min, float x_max, int bits);

#endif
