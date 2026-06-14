/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-07-09 21:22:21
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-07-12 17:41:29
 * @FilePath: \smartcare:\X15\stm32\my_example\RM\H7\DM-balanceV1\User\Bsp\bsp_usart.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef BSP_USART_H
#define BSP_USART_H

#include "main.h"
#include "string.h"

#include "robot_param.h"
#include "fifo.h"

#define BUFF_SIZE	25
#define CHASSIS_DATA_LENGTH 28

//云台收到的底盘数据,由串口发送到云台
typedef struct chassis_data
{
    //底盘速度信息
    float chassis_vx;
    float chassis_vy;
    float chassis_wz;
    
    //底盘陀螺仪信息
    float chassis_yaw;
    float chassis_pitch;
    float chassis_roll;

    //底盘超级电容电压
    float cap_voltage;

    //底盘模式
    uint8_t chassis_mode;

    //机器人ID
    uint8_t robot_id;
}chassis_data_t;

//底盘收到的云台数据
typedef struct gimbal_data
{
    //云台板陀螺仪参数
    float INS_yaw;
    float INS_pitch;
    float INS_roll;

    //云台电机参数（相对角度）
    float motor_yaw;
    float motor_pitch;

}gimbal_data_t;


#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

#define SBUS_HEAD 0X0F
#define SBUS_END 0X00
#define REMOTE_RC_OFFSET 1024
#define REMOTE_TOGGLE_DUAL_VAL 1024
#define REMOTE_TOGGLE_THRE_VAL_A 600
#define REMOTE_TOGGLE_THRE_VAL_B 1400
#define DEAD_AREA	120








extern chassis_data_t chassis_data;
extern gimbal_data_t gimbal;

extern float chassis_INS_yaw, chassis_INS_pitch, chassis_spin_speed;
extern uint8_t remote_buff[SBUS_RX_BUF_NUM];
extern uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];
extern uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
extern fifo_s_t referee_fifo;
extern uint8_t referee_fifo_ready;
extern uint8_t usart7_buf[ USART_RX_BUF_LENGHT ];//设置缓冲区
extern uint8_t usart10_buf[ USART_RX_BUF_LENGHT ];//设置缓冲区

//遥控器
typedef struct
{
    uint16_t online;
		uint32_t sbus_recever_time;

    struct
    {
        int16_t ch[10];
    } rc;

    struct
    {
        /* STICK VALUE */
        int16_t left_vert;
        int16_t left_hori;
        int16_t right_vert;
        int16_t right_hori;
    } joy;
		
		struct
		{
			//l1 l2 r2 r1
			uint8_t swa;//2-Stop
			uint8_t swb;//3-Stop
			uint8_t swc;//3-Stop
			uint8_t swd;//2-Stop
		} toggle;

    struct
    {
        /* VAR VALUE */
        float a;
        float b;
    } var;

    struct
    {
        /* KEY VALUE */
        uint8_t l;
				uint8_t r;
    } key;
} remoter_t;



extern remoter_t remoter;

extern void USART1_Transmit_DMA(uint8_t *pData, uint16_t Size);
extern void USART7_Transmit(uint8_t *pData, uint16_t Size);
extern void USART8_Transmit(uint8_t *pData, uint16_t Size);
extern void USART10_Transmit(uint8_t *pData, uint16_t Size);
extern void USART10_Transmit_IT(uint8_t *pData, uint16_t Size);
void USART1_Transmit(uint8_t *pData, uint16_t Size);
void USART1_Transmit_IT(uint8_t *pData, uint16_t Size);

#endif
