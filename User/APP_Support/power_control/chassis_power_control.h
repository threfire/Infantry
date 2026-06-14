/**
  * @file       chassis_power_control.h
  * @brief      底盘功率控制接口声明
  * @note       定义功率限制状态、超级电容模式和功率控制入口。
  */
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

#include "chassis_task.h"
#include "robot_param.h"

#ifndef LIMIT_MAX_MIN
#define LIMIT_MAX_MIN(val, max, min) ((val) > (max) ? (max) : ((val) < (min) ? (min) : (val)))
#endif

#define SET_POWER_VALUE 100.0f

typedef enum {
    SUPER_CAP_CHARGING = 0,
    SUPER_CAP_PREPARED,
    SUPER_CAP_USING,
} super_cap_mode_e;

typedef struct {
    float k_1;
    float k_2;
    float a;
    float K_Reduction;
    float cur_motorspeed_set[CHASSIS_MODULE_NUM];
    float now_motorspeed[CHASSIS_MODULE_NUM];
    float set_motorcurrent[CHASSIS_MODULE_NUM];
    float now_motorcurrent[CHASSIS_MODULE_NUM];
    float set_power;
    float P_origin;
    float P_bus;
    float P_in;
} PowerLimit_t;

extern PowerLimit_t PowerLimit;

void Current_RestraintRelation_Calc(PowerLimit_t *PowerLimit_Cur);
void Predict_Power(PowerLimit_t *PowerLimit_Pre, chassis_move_t *chassis_move);
void chassis_power_control(chassis_move_t *chassis_move);

#endif
