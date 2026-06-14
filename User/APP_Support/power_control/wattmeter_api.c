/**
  * @file       wattmeter_api.c
  * @brief      功率计反馈解析
  * @note       解析功率计 CAN 反馈并更新功率观测数据。
  */
#include "wattmeter_api.h"

volatile wattmeter_measure_t wattmeter_measure = {0.0f, 0.0f, 0.0f};

/**
  * @brief          解析功率计 CAN 反馈
  * @param[in]      can_id: CAN 标识符
  * @param[in]      can_rx_data: CAN 数据区指针
  * @param[in]      len: 数据长度，单位 byte
  * @retval         none
  */
void wattmeter_feedback_handle(uint16_t can_id, const uint8_t *can_rx_data, uint8_t len)
{
    uint16_t voltage_raw;
    uint16_t current_raw;

    if(can_id != WATTMETER_CAN_ID || can_rx_data == 0 || len < 4U)
    {
        return;
    }

    voltage_raw = (uint16_t)((uint16_t)can_rx_data[1] << 8) | can_rx_data[0];
    current_raw = (uint16_t)((uint16_t)can_rx_data[3] << 8) | can_rx_data[2];

    wattmeter_measure.voltage = (float)voltage_raw / 100.0f;
    wattmeter_measure.current = (float)current_raw / 100.0f;
    wattmeter_measure.power = wattmeter_measure.voltage * wattmeter_measure.current;
}
