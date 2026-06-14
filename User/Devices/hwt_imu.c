#include "hwt_imu.h"
#include "robot_param.h"
#include <string.h>

#define HWT_PI              PI
#define HWT_TWO_PI          (2.0f * HWT_PI)
#define HWT_DEG_TO_RAD      (HWT_PI / 180.0f)
#define HWT_RAD_TO_DEG      (180.0f / HWT_PI)
#define HWT_GYRO_RANGE_DPS  2000.0f
#define HWT_ANGLE_RANGE_DEG 180.0f
#define HWT_ACCEL_RANGE_G   16.0f
#define HWT_GRAVITY_MPS2    9.8f

uint8_t imu101_rx_buf[IMU101_RX_BUF_LEN];
uint8_t imu906_rx_buf[IMU906_RX_BUF_LEN];

/* 使用全局变量，方便调试器直接监测 */
hwt_imu_info_t hwt101_info;
hwt_imu_info_t hwt906_info;

typedef struct
{
    uint8_t frame[HWT_FRAME_LEN];
    uint8_t index;
} hwt_parser_t;

static hwt_parser_t s_hwt101_parser;
static hwt_parser_t s_hwt906_parser;

/* 101：底盘只需要 yaw，导出连续 yaw */
static float s_hwt101_chassis_yaw[1];

/* 906：云台导出连续 yaw 和 pitch */
static float s_hwt906_gimbal_angle[3];

/* 906：云台角速度 */
static float s_hwt906_gimbal_gyro[3];

/* 906：云台线加速度，单位 m/s^2 */
static float s_hwt906_gimbal_accel[3];

static int16_t hwt_read_s16_le(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[1] << 8) | (uint16_t)p[0]);
}

static uint8_t hwt_calc_sum(const uint8_t *frame)
{
    uint8_t i;
    uint8_t sum = 0;

    for (i = 0; i < (HWT_FRAME_LEN - 1u); i++)
    {
        sum = (uint8_t)(sum + frame[i]);
    }

    return sum;
}

static void hwt_update_yaw_total(hwt_angle_info_t *angle)
{
    float current_yaw_rad;
    float delta_yaw_rad;

    if (angle == 0)
    {
        return;
    }

    current_yaw_rad = angle->rad[HWT_AXIS_YAW];

    if (angle->yaw_init_flag == 0u)
    {
        angle->last_yaw_rad = current_yaw_rad;
        angle->yaw_total_rad = current_yaw_rad;
        angle->yaw_total_deg = current_yaw_rad * HWT_RAD_TO_DEG;
        angle->yaw_init_flag = 1u;
        return;
    }

    delta_yaw_rad = current_yaw_rad - angle->last_yaw_rad;

    if (delta_yaw_rad > HWT_PI)
    {
        delta_yaw_rad -= HWT_TWO_PI;
    }
    else if (delta_yaw_rad < -HWT_PI)
    {
        delta_yaw_rad += HWT_TWO_PI;
    }

    angle->yaw_total_rad += delta_yaw_rad;
    angle->yaw_total_deg = angle->yaw_total_rad * HWT_RAD_TO_DEG;
    angle->last_yaw_rad = current_yaw_rad;
}

static void hwt_update_export_data(hwt_imu_info_t *imu, uint8_t is_hwt906)
{
    if (is_hwt906 != 0u)
    {
        /* 云台角度输出：
         * [0] = 连续 yaw(rad)
         * [1] = pitch(rad)
         */
        s_hwt906_gimbal_angle[HWT_GIMBAL_YAW_INDEX] = imu->angle.yaw_total_rad;
        s_hwt906_gimbal_angle[HWT_GIMBAL_PITCH_INDEX] = imu->angle.rad[HWT_AXIS_PITCH];
        s_hwt906_gimbal_angle[HWT_GIMBAL_ROLL_INDEX] = imu->angle.rad[HWT_AXIS_ROLL];

        /* 云台角速度输出：
         * [0] = wx(rad/s)
         * [1] = wy(rad/s)
         * [2] = wz(rad/s)
         */
        s_hwt906_gimbal_gyro[HWT_AXIS_ROLL]  = imu->gyro.radps[HWT_AXIS_ROLL];
        s_hwt906_gimbal_gyro[HWT_AXIS_PITCH] = imu->gyro.radps[HWT_AXIS_PITCH];
        s_hwt906_gimbal_gyro[HWT_AXIS_YAW]   = imu->gyro.radps[HWT_AXIS_YAW];

        /* 906 的 0x51 帧线加速度输出：
         * [0] = ax(m/s^2)
         * [1] = ay(m/s^2)
         * [2] = az(m/s^2)
         */
        s_hwt906_gimbal_accel[HWT_AXIS_ROLL]  = imu->accel.mps2[HWT_AXIS_ROLL];
        s_hwt906_gimbal_accel[HWT_AXIS_PITCH] = imu->accel.mps2[HWT_AXIS_PITCH];
        s_hwt906_gimbal_accel[HWT_AXIS_YAW]   = imu->accel.mps2[HWT_AXIS_YAW];
    }
    else
    {
        /* 底盘导出 101 连续 yaw */
        s_hwt101_chassis_yaw[0] = imu->angle.yaw_total_rad;
    }
}

static void hwt_parse_gyro_frame(hwt_imu_info_t *imu, const uint8_t *frame)
{
    uint8_t axis;

    imu->gyro.raw[HWT_AXIS_ROLL]  = hwt_read_s16_le(&frame[2]);
    imu->gyro.raw[HWT_AXIS_PITCH] = hwt_read_s16_le(&frame[4]);
    imu->gyro.raw[HWT_AXIS_YAW]   = hwt_read_s16_le(&frame[6]);

    imu->gyro.voltage_raw = (uint16_t)(((uint16_t)frame[9] << 8) | (uint16_t)frame[8]);
    imu->gyro.voltage = ((float)imu->gyro.voltage_raw) / 100.0f;

    for (axis = 0; axis < 3u; axis++)
    {
        imu->gyro.dps[axis]   = ((float)imu->gyro.raw[axis]) / 32768.0f * HWT_GYRO_RANGE_DPS;
        imu->gyro.radps[axis] = imu->gyro.dps[axis] * HWT_DEG_TO_RAD;
    }

    imu->gyro.updated = 1u;
}

static void hwt_parse_accel_frame(hwt_imu_info_t *imu, const uint8_t *frame)
{
    uint8_t axis;

    imu->accel.raw[HWT_AXIS_ROLL]  = hwt_read_s16_le(&frame[2]);
    imu->accel.raw[HWT_AXIS_PITCH] = hwt_read_s16_le(&frame[4]);
    imu->accel.raw[HWT_AXIS_YAW]   = hwt_read_s16_le(&frame[6]);
    imu->accel.temp_raw = hwt_read_s16_le(&frame[8]);
    imu->accel.temp_c = ((float)imu->accel.temp_raw) / 100.0f;

    for (axis = 0; axis < 3u; axis++)
    {
        imu->accel.g[axis] = ((float)imu->accel.raw[axis]) / 32768.0f * HWT_ACCEL_RANGE_G;
        imu->accel.mps2[axis] = imu->accel.g[axis] * HWT_GRAVITY_MPS2;
    }

    imu->accel.updated = 1u;
}

static void hwt_parse_angle_frame(hwt_imu_info_t *imu, const uint8_t *frame)
{
    uint8_t axis;

    imu->angle.raw[HWT_AXIS_ROLL]  = hwt_read_s16_le(&frame[2]);
    imu->angle.raw[HWT_AXIS_PITCH] = hwt_read_s16_le(&frame[4]);
    imu->angle.raw[HWT_AXIS_YAW]   = hwt_read_s16_le(&frame[6]);

    imu->angle.version = (uint16_t)(((uint16_t)frame[9] << 8) | (uint16_t)frame[8]);

    for (axis = 0; axis < 3u; axis++)
    {
        imu->angle.deg[axis] = ((float)imu->angle.raw[axis]) / 32768.0f * HWT_ANGLE_RANGE_DEG;
        imu->angle.rad[axis] = imu->angle.deg[axis] * HWT_DEG_TO_RAD;
    }

    hwt_update_yaw_total(&imu->angle);
    imu->angle.updated = 1u;
}

static void hwt_handle_frame(hwt_imu_info_t *imu, const uint8_t *frame, uint8_t is_hwt906)
{
    if (hwt_calc_sum(frame) != frame[HWT_FRAME_LEN - 1u])
    {
        imu->bad_frame_count++;
        return;
    }

    imu->ok_frame_count++;

    switch (frame[1])
    {
    case HWT_TYPE_ACCEL:
        if (is_hwt906 != 0u)
        {
            hwt_parse_accel_frame(imu, frame);
        }
        break;

    case HWT_TYPE_GYRO:
        hwt_parse_gyro_frame(imu, frame);
        break;

    case HWT_TYPE_ANGLE:
        hwt_parse_angle_frame(imu, frame);
        break;

    default:
        break;
    }

    hwt_update_export_data(imu, is_hwt906);
}

static void hwt_push_byte(hwt_parser_t *parser, hwt_imu_info_t *imu, uint8_t is_hwt906, uint8_t byte)
{
    if (parser->index == 0u)
    {
        if (byte == HWT_FRAME_HEAD)
        {
            parser->frame[0] = byte;
            parser->index = 1u;
        }
        return;
    }

    if ((parser->index == 1u) && (byte == HWT_FRAME_HEAD))
    {
        parser->frame[0] = byte;
        parser->index = 1u;
        return;
    }

    parser->frame[parser->index] = byte;
    parser->index++;

    if (parser->index >= HWT_FRAME_LEN)
    {
        hwt_handle_frame(imu, parser->frame, is_hwt906);
        parser->index = 0u;
    }
}

static void hwt_parse_stream(hwt_parser_t *parser, hwt_imu_info_t *imu, uint8_t is_hwt906, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == 0)
    {
        return;
    }

    for (i = 0u; i < len; i++)
    {
        hwt_push_byte(parser, imu, is_hwt906, data[i]);
    }
}

void hwt_imu_init(void)
{
    memset(&s_hwt101_parser, 0, sizeof(s_hwt101_parser));
    memset(&s_hwt906_parser, 0, sizeof(s_hwt906_parser));

    memset(&hwt101_info, 0, sizeof(hwt101_info));
    memset(&hwt906_info, 0, sizeof(hwt906_info));

    memset(s_hwt101_chassis_yaw, 0, sizeof(s_hwt101_chassis_yaw));
    memset(s_hwt906_gimbal_angle, 0, sizeof(s_hwt906_gimbal_angle));
    memset(s_hwt906_gimbal_gyro, 0, sizeof(s_hwt906_gimbal_gyro));
    memset(s_hwt906_gimbal_accel, 0, sizeof(s_hwt906_gimbal_accel));
}

void hwt101_rx_parse(const uint8_t *data, uint16_t len)
{
    hwt_parse_stream(&s_hwt101_parser, &hwt101_info, 0u, data, len);
}

void hwt906_rx_parse(const uint8_t *data, uint16_t len)
{
    hwt_parse_stream(&s_hwt906_parser, &hwt906_info, 1u, data, len);
}

const hwt_imu_info_t *hwt101_get_info(void)
{
    return &hwt101_info;
}

const hwt_imu_info_t *hwt906_get_info(void)
{
    return &hwt906_info;
}

float hwt101_get_yaw_rad(void)
{
    return hwt101_info.angle.rad[HWT_AXIS_YAW];
}

float hwt101_get_yaw_deg(void)
{
    return hwt101_info.angle.deg[HWT_AXIS_YAW];
}

float hwt101_get_yaw_total_rad(void)
{
    return hwt101_info.angle.yaw_total_rad;
}

float hwt101_get_yaw_total_deg(void)
{
    return hwt101_info.angle.yaw_total_deg;
}

float hwt906_get_yaw_rad(void)
{
    return hwt906_info.angle.rad[HWT_AXIS_YAW];
}

float hwt906_get_yaw_total_rad(void)
{
    return hwt906_info.angle.yaw_total_rad;
}

float hwt906_get_yaw_total_deg(void)
{
    return hwt906_info.angle.yaw_total_deg;
}

float hwt906_get_pitch_rad(void)
{
    return hwt906_info.angle.rad[HWT_AXIS_PITCH];
}

float hwt906_get_yaw_deg(void)
{
    return hwt906_info.angle.deg[HWT_AXIS_YAW];
}

float hwt906_get_pitch_deg(void)
{
    return hwt906_info.angle.deg[HWT_AXIS_PITCH];
}

const float *hwt101_get_chassis_yaw_point(void)
{
    return s_hwt101_chassis_yaw;
}

const float *hwt906_get_gimbal_angle_point(void)
{
    return s_hwt906_gimbal_angle;
}

const float *hwt906_get_gimbal_gyro_point(void)
{
    return s_hwt906_gimbal_gyro;
}

const float *hwt906_get_gimbal_accel_point(void)
{
    return s_hwt906_gimbal_accel;
}

void hwt101_clear_update_flag(void)
{
    hwt101_info.angle.updated = 0u;
    hwt101_info.gyro.updated = 0u;
}

void hwt906_clear_update_flag(void)
{
    hwt906_info.angle.updated = 0u;
    hwt906_info.gyro.updated = 0u;
    hwt906_info.accel.updated = 0u;
}

const float *get_INS_angle_point(void)
{
    return hwt906_get_gimbal_angle_point();
}

const float *get_gyro_data_point(void)
{
    return hwt906_get_gimbal_gyro_point();
}

const float *get_accel_data_point(void)
{
    return hwt906_get_gimbal_accel_point();
}
