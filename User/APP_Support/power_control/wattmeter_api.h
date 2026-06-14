/**
  * @file       wattmeter_api.h
  * @brief      功率计接口声明
  * @note       声明功率计反馈数据结构和解析接口。
  */
#ifndef WATTMETER_API_H
#define WATTMETER_API_H

#include <stdint.h>

#define WATTMETER_CAN_ID 0x212U

typedef struct
{
    float voltage;
    float current;
    float power;
} wattmeter_measure_t;

extern volatile wattmeter_measure_t wattmeter_measure;

void wattmeter_feedback_handle(uint16_t can_id, const uint8_t *can_rx_data, uint8_t len);

#endif
