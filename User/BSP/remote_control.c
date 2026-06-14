/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  **/
#include "remote_control.h"
#include "main.h"

#include "bsp_usart.h"
#include "string.h"

//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart5_rx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
//static uint8_t gimbal_data[10]={0};
//取正函数
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;


/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_buff, SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
void sbus_to_rc(uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl, remoter_t *remoter) //两种遥控器融合
{		
		if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
		
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
		rc_ctrl->last_fdb = HAL_GetTick();
		
}
void sbus_to_rc_uper(uint8_t *buf, RC_ctrl_t *rc_ctrl, remoter_t *remoter)
{		
    // 1. 首先进行帧头和帧尾检查
    if ((buf[0] != SBUS_HEAD) || (buf[24] != SBUS_END))
        return;
    // 2. 解析在线状态
    if (buf[23] == 0x0C)
        remoter->online = 0;
    else
        remoter->online = 1;
    // 3. 解析10个通道到remoter结构体（保持原解析方式）
    remoter->rc.ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
    remoter->rc.ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
    remoter->rc.ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
    remoter->rc.ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
    remoter->rc.ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
    remoter->rc.ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
    remoter->rc.ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
    remoter->rc.ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    remoter->rc.ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
    remoter->rc.ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);

    // 4. 计算摇杆值（使用赋值操作）
    remoter->joy.right_hori = remoter->rc.ch[3] - REMOTE_RC_OFFSET;   // 右摇杆左右（原来ch[0]）
    remoter->joy.right_vert = remoter->rc.ch[2] - REMOTE_RC_OFFSET;   // 右摇杆前进后退（原来ch[1]）
    remoter->joy.left_hori = remoter->rc.ch[0] - REMOTE_RC_OFFSET;    // 左摇杆左右（原来ch[2]）
    remoter->joy.left_vert = remoter->rc.ch[1] - REMOTE_RC_OFFSET;    // 左摇杆前进后退（原来ch[3]）
    
    // 5. 死区处理
    if( remoter->joy.left_hori > -DEAD_AREA && remoter->joy.left_hori < DEAD_AREA)
        remoter->joy.left_hori = 0;
    if( remoter->joy.right_hori > -DEAD_AREA && remoter->joy.right_hori < DEAD_AREA)
        remoter->joy.right_hori = 0;
    if( remoter->joy.right_vert > -DEAD_AREA && remoter->joy.right_vert < DEAD_AREA)
        remoter->joy.right_vert = 0;
    
    //对应原来左拨杆
    if(remoter->rc.ch[4] < REMOTE_TOGGLE_THRE_VAL_A)
    {
			rc_ctrl->rc.s[1] = 1;  // 位置上
    }
    else if(remoter->rc.ch[4] >= REMOTE_TOGGLE_THRE_VAL_A && remoter->rc.ch[4] <= REMOTE_TOGGLE_THRE_VAL_B)
    {
        rc_ctrl->rc.s[1] = 3;  // 位置中
    }
    else if(remoter->rc.ch[4] >= REMOTE_TOGGLE_THRE_VAL_B)
    {
        rc_ctrl->rc.s[1] = 2;  // 位置3
    }
    
    //对应原来右拨杆
    if(remoter->rc.ch[5] < REMOTE_TOGGLE_THRE_VAL_A)
    {
			rc_ctrl->rc.s[0] = 1;  // 位置上
    }
    else if(remoter->rc.ch[5] >= REMOTE_TOGGLE_THRE_VAL_A && remoter->rc.ch[5] <= REMOTE_TOGGLE_THRE_VAL_B)
    {
        rc_ctrl->rc.s[0] = 3;  // 位置中
    }
    else if(remoter->rc.ch[5] >= REMOTE_TOGGLE_THRE_VAL_B)
    {
        rc_ctrl->rc.s[0] = 2;  // 位置下
    }
    // 9. 更新时间戳
    remoter->sbus_recever_time = HAL_GetTick();

    // 10. 将remoter的值赋给RC_ctrl_t结构体
    //这里需要根据两个结构体之间的对应关系进行赋值
    
    //直接使用remoter解析后的原始值
    rc_ctrl->rc.ch[0] = -remoter->joy.right_hori;  // 右摇杆左右
    rc_ctrl->rc.ch[1] = remoter->joy.right_vert;  // 右摇杆前进后退
    rc_ctrl->rc.ch[2] = -remoter->joy.left_hori;  // 左摇杆左右
    rc_ctrl->rc.ch[3] = -remoter->joy.left_vert;  // 左摇杆前进后退
}
/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          通过usart1发送sbus数据,在usart3_IRQHandle调用
  * @param[in]      sbus: sbus数据, 18字节
  * @retval         none
  */
void sbus_to_usart1(uint8_t *sbus)
{
   static uint8_t usart_tx_buf[20];
   static uint8_t i =0;
   usart_tx_buf[0] = 0xA6;
   memcpy(usart_tx_buf + 1, sbus, 18);
   for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
   {
       usart_tx_buf[19] += usart_tx_buf[i];
   }
   USART1_Transmit_DMA(usart_tx_buf, 20);
}

