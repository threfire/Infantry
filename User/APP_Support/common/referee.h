/**
  * @file       referee.h
  * @brief      裁判系统数据结构与接口声明
  * @note       定义裁判协议数据结构和业务查询接口。
  */
#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"
#include "protocol.h"

extern uint32_t g_referee_last_rx_ms;

void init_referee_data(void);
void init_referee_struct_data(void);
void referee_handle_data(uint8_t *frame);
void referee_data_solve(uint8_t *frame);
uint8_t referee_data_available(uint32_t timeout_ms);

uint8_t get_robot_id(void);
uint16_t get_shooter_barrel_heat_limit(void);
uint16_t get_chassis_power_limit(void);
uint16_t get_buffer_energy(void);
uint16_t get_shooter_17mm_heat(void);
uint16_t get_shooter_42mm_heat(void);
void get_chassis_power_and_buffer(uint16_t *power_limit, uint16_t *buffer);
void get_shoot_heat0_limit_and_heat0(uint16_t *heat_limit, uint16_t *heat);
void get_shoot_heat1_limit_and_heat1(uint16_t *heat_limit, uint16_t *heat);

#endif
