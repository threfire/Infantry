#ifndef HWT_IMU_H
#define HWT_IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HWT_FRAME_HEAD      ((uint8_t)0x55)
#define HWT_FRAME_LEN       11u

#define HWT_TYPE_ACCEL      ((uint8_t)0x51)
#define HWT_TYPE_GYRO       ((uint8_t)0x52)
#define HWT_TYPE_ANGLE      ((uint8_t)0x53)

#define HWT_AXIS_ROLL       0u
#define HWT_AXIS_PITCH      1u
#define HWT_AXIS_YAW        2u

/* 云台角度数组索引：
 * [0] = yaw(rad)
 * [1] = pitch(rad)
 */
#define HWT_GIMBAL_YAW_INDEX    0u
#define HWT_GIMBAL_PITCH_INDEX  1u
#define HWT_GIMBAL_ROLL_INDEX   2u

#define IMU101_RX_BUF_LEN  64
#define IMU906_RX_BUF_LEN  64

extern uint8_t imu101_rx_buf[IMU101_RX_BUF_LEN];
extern uint8_t imu906_rx_buf[IMU906_RX_BUF_LEN];

typedef struct
{
    int16_t raw[3];
    float deg[3];
    float rad[3];

    /* yaw 连续角，跨 +/-180 度边界后展开 */
    float yaw_total_deg;
    float yaw_total_rad;
    float last_yaw_rad;
    uint8_t yaw_init_flag;

    uint16_t version;
    uint8_t updated;
} hwt_angle_info_t;

typedef struct
{
    int16_t raw[3];
    float dps[3];
    float radps[3];
    uint16_t voltage_raw;
    float voltage;
    uint8_t updated;
} hwt_gyro_info_t;

typedef struct
{
    int16_t raw[3];
    float g[3];
    float mps2[3];
    int16_t temp_raw;
    float temp_c;
    uint8_t updated;
} hwt_accel_info_t;

typedef struct
{
    hwt_angle_info_t angle;
    hwt_gyro_info_t gyro;
    hwt_accel_info_t accel;
    uint32_t ok_frame_count;
    uint32_t bad_frame_count;
} hwt_imu_info_t;

/**
  * @brief          IMU 模块初始化
  * @param[in]      none
  * @retval         none
  */
void hwt_imu_init(void);

/**
  * @brief          101 串口接收数据解包
  * @param[in]      data: 串口收到的数据缓冲区
  * @param[in]      len:  数据长度
  * @retval         none
  */
void hwt101_rx_parse(const uint8_t *data, uint16_t len);

/**
  * @brief          906 串口接收数据解包
  * @param[in]      data: 串口收到的数据缓冲区
  * @param[in]      len:  数据长度
  * @retval         none
  */
void hwt906_rx_parse(const uint8_t *data, uint16_t len);

/**
  * @brief          获取 101 完整信息
  * @param[in]      none
  * @retval         101 信息结构体指针
  */
const hwt_imu_info_t *hwt101_get_info(void);

/**
  * @brief          获取 906 完整信息
  * @param[in]      none
  * @retval         906 信息结构体指针
  */
const hwt_imu_info_t *hwt906_get_info(void);

/**
  * @brief          获取 101 原始 yaw
  * @param[in]      none
  * @retval         yaw(rad)，范围约 [-pi, pi]
  */
float hwt101_get_yaw_rad(void);

/**
  * @brief          获取 101 原始 yaw
  * @param[in]      none
  * @retval         yaw(deg)，范围约 [-180, 180]
  */
float hwt101_get_yaw_deg(void);

/**
  * @brief          获取 101 连续 yaw
  * @param[in]      none
  * @retval         yaw(rad)
  */
float hwt101_get_yaw_total_rad(void);

/**
  * @brief          获取 101 连续 yaw
  * @param[in]      none
  * @retval         yaw(deg)
  */
float hwt101_get_yaw_total_deg(void);

/**
  * @brief          获取 906 原始 yaw
  * @param[in]      none
  * @retval         yaw(rad)，范围约 [-pi, pi]
  */
float hwt906_get_yaw_rad(void);

/**
  * @brief          获取 906 连续 yaw
  * @param[in]      none
  * @retval         yaw(rad)
  */
float hwt906_get_yaw_total_rad(void);

/**
  * @brief          获取 906 连续 yaw
  * @param[in]      none
  * @retval         yaw(deg)
  */
float hwt906_get_yaw_total_deg(void);

/**
  * @brief          获取 906 pitch
  * @param[in]      none
  * @retval         pitch(rad)
  */
float hwt906_get_pitch_rad(void);

/**
  * @brief          获取 906 原始 yaw
  * @param[in]      none
  * @retval         yaw(deg)，范围约 [-180, 180]
  */
float hwt906_get_yaw_deg(void);

/**
  * @brief          获取 906 pitch
  * @param[in]      none
  * @retval         pitch(deg)
  */
float hwt906_get_pitch_deg(void);

/**
  * @brief          获取底盘 yaw 指针，来自 101
  * @param[in]      none
  * @retval         单元素数组指针，point[0] = 连续 yaw(rad)
  */
const float *hwt101_get_chassis_yaw_point(void);

/**
  * @brief          获取云台角度指针，来自 906
  * @param[in]      none
  * @retval         point[0] = 连续 yaw(rad), point[1] = pitch(rad)
  */
const float *hwt906_get_gimbal_angle_point(void);

/**
  * @brief          获取云台角速度指针，来自 906
  * @param[in]      none
  * @retval         point[0] = wx(rad/s), point[1] = wy(rad/s), point[2] = wz(rad/s)
  */
const float *hwt906_get_gimbal_gyro_point(void);

/**
  * @brief          获取云台线加速度指针，来自 906
  * @param[in]      none
  * @retval         point[0] = ax(m/s^2), point[1] = ay(m/s^2), point[2] = az(m/s^2)
  */
const float *hwt906_get_gimbal_accel_point(void);

/**
  * @brief          清除 101 更新标志
  * @param[in]      none
  * @retval         none
  */
void hwt101_clear_update_flag(void);

/**
  * @brief          清除 906 更新标志
  * @param[in]      none
  * @retval         none
  */
void hwt906_clear_update_flag(void);

/**
  * @brief          提供给云台框架的角度钩子
  * @param[in]      none
  * @retval         906 角度指针，point[0] = 连续 yaw(rad), point[1] = pitch(rad)
  */
const float *get_INS_angle_point(void);

/**
  * @brief          提供给云台框架的角速度钩子
  * @param[in]      none
  * @retval         906 角速度指针，point[0] = wx(rad/s), point[1] = wy(rad/s), point[2] = wz(rad/s)
  */
const float *get_gyro_data_point(void);

/**
  * @brief          提供给云台框架的线加速度钩子
  * @param[in]      none
  * @retval         906 线加速度指针，point[0] = ax, point[1] = ay, point[2] = az
  */
const float *get_accel_data_point(void);

#ifdef __cplusplus
}
#endif

#endif
