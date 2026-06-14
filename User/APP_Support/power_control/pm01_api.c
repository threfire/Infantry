/**
  * @file       pm01_api.c
  * @brief      PM01 超级电容模块协议接口
  * @note       发送功率/电压/电流配置命令并解析模块反馈。
  */
#include "pm01_api.h"

#include "bsp_fdcan.h"
#include "chassis_power_control.h"
#include "robot_param.h"

volatile pm01_od_t pm01_od;
volatile uint16_t pm01_access_id;
volatile uint16_t pm01_response_flg;

uint16_t g_cmd_set = 2U;
uint16_t g_power_set = (uint16_t)(SET_POWER_VALUE * 100.0f);
uint16_t g_vout_set = 2600U;
uint16_t g_iout_set = 600U;

static void pm01_send_u16(uint16_t can_id, uint16_t value, uint8_t save_flg)
{
    uint8_t data[4];

    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
    data[2] = 0U;
    data[3] = (save_flg == 0x01U) ? 1U : 0U;
    pm01_access_id = can_id;
    canx_send_data(&hfdcan1, can_id, data, 4U);
}

/**
  * @brief          发送 PM01 原始配置命令
  * @param[in]      new_cmd: 配置命令字
  * @param[in]      save_flg: 1 保存到模块，0 临时生效
  * @retval         none
  */
void pm01_cmd_send(uint16_t new_cmd, uint8_t save_flg)
{
    pm01_send_u16(0x600U, new_cmd, save_flg);
}

/**
  * @brief          设置 PM01 输出功率
  * @param[in]      new_power: 功率设置值
  * @param[in]      save_flg: 1 保存到模块，0 临时生效
  * @retval         none
  */
void pm01_power_set(uint16_t new_power, uint8_t save_flg)
{
    pm01_send_u16(0x601U, new_power, save_flg);
}

/**
  * @brief          设置 PM01 输出电压
  * @param[in]      new_voltage: 电压设置值
  * @param[in]      save_flg: 1 保存到模块，0 临时生效
  * @retval         none
  */
void pm01_voltage_set(uint16_t new_voltage, uint8_t save_flg)
{
    pm01_send_u16(0x602U, new_voltage, save_flg);
}

/**
  * @brief          设置 PM01 输出电流
  * @param[in]      new_current: 电流设置值
  * @param[in]      save_flg: 1 保存到模块，0 临时生效
  * @retval         none
  */
void pm01_current_set(uint16_t new_current, uint8_t save_flg)
{
    pm01_send_u16(0x603U, new_current, save_flg);
}

/**
  * @brief          PM01 访问轮询
  * @note           周期发送查询帧，驱动 PM01 反馈更新。
  * @retval         none
  */
void pm01_access_poll(void)
{
#if defined(ROBOT_CAP) && (ROBOT_CAP == Cap_on)
    static uint16_t tick = 0U;

    tick++;
    if (tick >= 100U)
    {
        tick = 0U;
        pm01_cmd_send(g_cmd_set, 0U);
        pm01_power_set(g_power_set, 0U);
        pm01_voltage_set(g_vout_set, 0U);
        pm01_current_set(g_iout_set, 0U);
    }
#endif
}

/**
  * @brief          解析 PM01 CAN 反馈
  * @param[in]      can_id: CAN 标识符
  * @param[in]      can_rx_data: CAN 数据区指针
  * @retval         none
  */
void pm01_response_handle(uint16_t can_id, const uint8_t *can_rx_data)
{
    uint16_t value0;
    uint16_t value1;
    uint16_t value2;

    if (can_rx_data == 0)
    {
        return;
    }

    value0 = (uint16_t)((uint16_t)can_rx_data[0] << 8) | can_rx_data[1];
    value1 = (uint16_t)((uint16_t)can_rx_data[2] << 8) | can_rx_data[3];
    value2 = (uint16_t)((uint16_t)can_rx_data[4] << 8) | can_rx_data[5];
    pm01_response_flg = (pm01_access_id == can_id) ? 1U : 0U;

    switch (can_id)
    {
        case 0x600U:
            pm01_od.ccr = value0;
            break;
        case 0x601U:
            pm01_od.p_set = value0;
            break;
        case 0x602U:
            pm01_od.v_set = value0;
            break;
        case 0x603U:
            pm01_od.i_set = value0;
            break;
        case 0x610U:
            pm01_od.sta_code.all = value0;
            pm01_od.err_code = value1;
            break;
        case 0x611U:
            pm01_od.p_in = (int16_t)value0;
            pm01_od.v_in = (int16_t)value1;
            pm01_od.i_in = (int16_t)value2;
            break;
        case 0x612U:
            pm01_od.p_out = (int16_t)value0;
            pm01_od.v_out = (int16_t)value1;
            pm01_od.i_out = (int16_t)value2;
            break;
        case 0x613U:
            pm01_od.temp = (int16_t)value0;
            pm01_od.total_time = value1;
            pm01_od.run_time = value2;
            break;
        default:
            break;
    }
}
