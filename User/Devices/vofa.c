#include "vofa.h"
#include "bsp_fdcan.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "usart.h"
#include <math.h>

#ifndef VOFA_SERVICE_CHASSIS_MOTOR_INDEX
#define VOFA_SERVICE_CHASSIS_MOTOR_INDEX 1U
#endif

static VOFA_JustFloatFrame_t s_vofa_frame =
{
    .tail = {0x00, 0x00, 0x80, 0x7F}
};

static void VOFA_Send6(float ch0, float ch1, float ch2, float ch3, float ch4, float ch5)
{
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    s_vofa_frame.fdata[0] = ch0;
    s_vofa_frame.fdata[1] = ch1;
    s_vofa_frame.fdata[2] = ch2;
    s_vofa_frame.fdata[3] = ch3;
    s_vofa_frame.fdata[4] = ch4;
    s_vofa_frame.fdata[5] = ch5;

    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&s_vofa_frame, sizeof(s_vofa_frame));
}

void VOFA_SendChassisMotorMeasure(uint8_t motor_idx)
{
    const motor_measure_t *motor;

    if (motor_idx >= VOFA_CHASSIS_MOTOR_COUNT)
    {
        return;
    }

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    motor = &CHASSIS_MOTOR_MEASURE[motor_idx];

    s_vofa_frame.fdata[0] = (float)motor->ecd;
    s_vofa_frame.fdata[1] = chassis_move.chassis_3508[motor_idx].speed_rad_s;
    s_vofa_frame.fdata[2] = chassis_move.chassis_3508[motor_idx].given_current_a;
    s_vofa_frame.fdata[3] = PowerLimit.P_bus;
    s_vofa_frame.fdata[4] = (float)motor->last_ecd;
    s_vofa_frame.fdata[5] = PowerLimit.P_origin;

    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&s_vofa_frame, sizeof(s_vofa_frame));
}

void VOFA_SendChassisPowerDebug(uint8_t motor_idx)
{
    if (motor_idx >= VOFA_CHASSIS_MOTOR_COUNT)
    {
        return;
    }

    VOFA_Send6(PowerLimit.set_power,
               PowerLimit.P_origin,
               PowerLimit.K_Reduction,
               chassis_move.model_3508_out[motor_idx],
               (float)chassis_move.chassis_3508[motor_idx].give_current,
               PowerLimit.P_bus);
}

void VOFA_SendChassisPowerCurrentDebug(void)
{
    VOFA_Send6(PowerLimit.P_origin,
               chassis_move.chassis_3508[0].given_current_a,
               chassis_move.chassis_3508[1].given_current_a,
               chassis_move.chassis_3508[2].given_current_a,
               chassis_move.chassis_3508[3].given_current_a,
               PowerLimit.P_bus);
}

void VOFA_SendChassisSpeedAccel(void)
{
    VOFA_Send6(chassis_move.vx_plan,
               chassis_move.vy_plan,
               chassis_move.wz_plan,
               chassis_move.vx_plan_accel,
               chassis_move.vy_plan_accel,
               chassis_move.wz_plan_accel);
}

void VOFA_SendChassisRealSpeedCurrent(void)
{
    VOFA_Send6(chassis_move.vx,
               chassis_move.vy,
               chassis_move.chassis_3508[0].given_current_a,
               chassis_move.chassis_3508[1].given_current_a,
               chassis_move.chassis_3508[2].given_current_a,
               chassis_move.chassis_3508[3].given_current_a);
}

void VOFA_SendChassisAnglePidDebug(void)
{
    float target = chassis_move.chassis_angle_pid.set;
    float actual = chassis_move.chassis_angle_pid.fdb;
    float error;
    float actual_vel = chassis_move.chassis_yaw_rate;

    if (chassis_move.chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        target = chassis_move.chassis_relative_angle_set;
        actual = chassis_move.chassis_relative_angle;
        actual_vel = chassis_move.chassis_relative_angle_vel;
    }
    else if (chassis_move.chassis_mode == CHASSIS_VECTOR_YAW_HOLD)
    {
        target = chassis_move.chassis_yaw_set;
        actual = chassis_move.chassis_yaw;
        actual_vel = chassis_move.chassis_yaw_rate;
    }

    error = target - actual;
    if (fabsf(error) > PI)
    {
        error += (error > 0.0f) ? (-2.0f * PI) : (2.0f * PI);
    }

    VOFA_Send6(target,
               actual,
               error,
               chassis_move.chassis_angle_pid.Pout,
               chassis_move.chassis_angle_pid.Dout,
               actual_vel);
}

void VOFA_SendChassisTranslatePairDebug(void)
{
    uint8_t motor_a = WHEEL_REAR_205;
    uint8_t motor_b = WHEEL_FRONT_207;

    if (fabsf(chassis_move.vx_plan) > fabsf(chassis_move.vy_plan))
    {
        motor_a = WHEEL_RIGHT_206;
        motor_b = WHEEL_LEFT_208;
    }

    VOFA_Send6(chassis_move.chassis_3508[motor_a].speed_set,
               chassis_move.chassis_3508[motor_a].speed,
               (float)chassis_move.chassis_3508[motor_a].give_current,
               chassis_move.chassis_3508[motor_b].speed_set,
               chassis_move.chassis_3508[motor_b].speed,
               (float)chassis_move.chassis_3508[motor_b].give_current);
}

void VOFA_SendChassisMotionDebug(void)
{
    float angle = chassis_move.chassis_yaw;
    float target_angle = chassis_move.chassis_yaw_set;

    if (chassis_move.chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        angle = chassis_move.chassis_relative_angle;
        target_angle = chassis_move.chassis_relative_angle_set;
    }

    VOFA_Send6(chassis_move.vx_plan,
               chassis_move.vy_plan,
               chassis_move.body_ff_ax,
               chassis_move.body_ff_ay,
               angle,
               target_angle);
}

void VOFA_SendGimbalFric(void)
{
    float current_avg;

    current_avg = (shoot_task_control.fric1.give_current_a +
                   shoot_task_control.fric2.give_current_a +
                   shoot_task_control.fric3.give_current_a) / 3.0f;

    VOFA_Send6(shoot_task_control.fric1.speed_rpm,
               shoot_task_control.fric2.speed_rpm,
               shoot_task_control.fric3.speed_rpm,
               current_avg,
               shoot_task_control.bullet_speed_min_avg_rpm,
               shoot_task_control.estimated_bullet_speed_mps);
}

void VOFA_SendGimbalYaw(void)
{
    const gimbal_motor_t *yaw = &gimbal_control.gimbal_yaw_motor;

    VOFA_Send6(yaw->relative_angle_set,
               yaw->relative_angle,
               yaw->absolute_angle_set,
               yaw->absolute_angle,
               yaw->gyro,
               yaw->current_set);
}

void VOFA_SendGimbalPitch(void)
{
    const gimbal_motor_t *pitch = &gimbal_control.gimbal_pitch_motor;

    VOFA_Send6(pitch->relative_angle_set,
               pitch->relative_angle,
               pitch->absolute_angle_set,
               pitch->absolute_angle,
               pitch->gyro,
               pitch->current_set);
}

void VOFA_SendGimbalYawPitchHalf(void)
{
    const gimbal_motor_t *yaw = &gimbal_control.gimbal_yaw_motor;
    const gimbal_motor_t *pitch = &gimbal_control.gimbal_pitch_motor;

    VOFA_Send6(yaw->relative_angle_set,
               yaw->relative_angle,
               yaw->static_friction_comp,
               pitch->relative_angle_set,
               pitch->relative_angle,
               pitch->static_friction_comp);
}

void VOFA_SendGimbalStrum(void)
{
    const MITMeasure_t *strum = &MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX];

    VOFA_Send6(strum->fdb.pos,
               strum->set.POS,
               strum->fdb.vel,
               strum->fdb.tor,
               strum->set.TOR,
               strum->fdb.t_motor);
}

void VOFA_ServiceSend(void)
{
    VOFA_SendChassisRealSpeedCurrent();
}
