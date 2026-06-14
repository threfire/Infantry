/**
  * @file       pm01_api.h
  * @brief      PM01 超级电容模块接口声明
  * @note       声明配置命令、访问轮询和反馈解析接口。
  */
#ifndef PM01_API_H
#define PM01_API_H

#include <stdint.h>

typedef union
{
    uint16_t all;
    struct
    {
        uint16_t rdy : 1;
        uint16_t run : 1;
        uint16_t alm : 1;
        uint16_t pwr : 1;
        uint16_t load : 1;
        uint16_t cc : 1;
        uint16_t cv : 1;
        uint16_t cw : 1;
        uint16_t revd : 7;
        uint16_t flt : 1;
    } bit;
} csr_t;

typedef struct
{
    uint16_t ccr;
    uint16_t p_set;
    uint16_t v_set;
    uint16_t i_set;
    csr_t sta_code;
    uint16_t err_code;
    int16_t v_in;
    int16_t i_in;
    int16_t p_in;
    int16_t v_out;
    int16_t i_out;
    int16_t p_out;
    int16_t temp;
    uint16_t total_time;
    uint16_t run_time;
} pm01_od_t;

extern volatile pm01_od_t pm01_od;

void pm01_cmd_send(uint16_t new_cmd, uint8_t save_flg);
void pm01_voltage_set(uint16_t new_voltage, uint8_t save_flg);
void pm01_current_set(uint16_t new_current, uint8_t save_flg);
void pm01_power_set(uint16_t new_power, uint8_t save_flg);
void pm01_access_poll(void);
void pm01_response_handle(uint16_t can_id, const uint8_t *can_rx_data);

#endif
