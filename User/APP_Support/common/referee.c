/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee.c
  * @brief     	裁判系统 接收到的数据的转移 与 某些数据获取
	*
	*     000000000000000     00               00    00     00   00         00 
	*           00     0      00                00    00   00     00       00  
	*       00  00000        00000000000000      00  000000000   00000000000000
	*       00  00          00           00    00    000000000     00     00   
	*      00000000000     00  000000    00     000     00           000000    
	*     00    00   0000     00    00   00      00   000000           00      
	*       00000000000       00    00   00           000000           00      
	*       0   00    0       0000000 00 00       00    00       00000000000000
	*       00000000000       00       000       00 00000000000        00      
	*           00            00                000 00000000000        00      
	*           00  00        00          0    000      00             00      
	*      000000000000        00        000  000       00          00 00      
	*       00        00        000000000000            00            00       
	********************************************************************************/

#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

game_status_t             				game_status;
game_result_t             				game_result;
game_robot_HP_t           		    game_robot_HP;
event_data_t              		  	event_data;
referee_warning_t         		 		referee_warning;
dart_info_t               		 		dart_info;
robot_status_t            		  	robot_status;
power_heat_data_t         				power_heat_data;
robot_pos_t               				robot_pos;
buff_t                  					buff;
hurt_data_t               				hurt_data;
shoot_data_t              				shoot_data;
projectile_allowance_t    		 		projectile_allowance;
rfid_status_t             		    rfid_status;
dart_client_cmd_t         		   	dart_client_cmd;
ground_robot_position_t   		  	ground_robot_position;
radar_mark_data_t         				radar_mark_data;
sentry_info_t             		    sentry_info;
radar_info_t              		   	radar_info;
robot_interaction_data_t  		   	robot_interaction_data;
interaction_layer_delete_t		  	interaction_layer_delete;
interaction_figure_t      		  	interaction_figure;
interaction_figure_2_t    		  	interaction_figure_2;
interaction_figure_3_t    		  	interaction_figure_3;
interaction_figure_4_t    		  	interaction_figure_4;
client_custom_character_t 		   	client_custom_character;
sentry_cmd_t              		   	sentry_cmd;
radar_cmd_t             					radar_cmd;
map_command_t             		    map_command;
map_robot_data_t          				map_robot_data;
map_data_t                		 		map_data;
custom_info_t             		    custom_info;
custom_robot_data_t       		 		custom_robot_data;
robot_custom_data_t       		   	robot_custom_data;
robot_custom_data_2_t     		  	robot_custom_data_2;
robot_custom_data_3_t     		  	robot_custom_data_3;
custom_client_data_t      		  	custom_client_data;
enemy_pos_t               				enemy_pos;
enemy_hp_t              					enemy_hp;
enemy_ammo_t              				enemy_ammo;
enemy_macro_status_t							enemy_macro_status;
enemy_buff_t              				enemy_buff;
enemy_wave_key_t               		enemy_wave_key;
uint32_t g_referee_last_rx_ms = 0;


/**
  * @brief          清空裁判系统全部接收数据
  * @retval         none
  */
void init_referee_data(void)
{
    memset(&referee_receive_header,			0, sizeof(frame_header_struct_t			));
    memset(&referee_send_header, 				0, sizeof(frame_header_struct_t			));

    memset(&game_status,								0, sizeof(game_status_t             ));
    memset(&game_result,                0, sizeof(game_result_t             ));
    memset(&game_robot_HP,              0, sizeof(game_robot_HP_t           ));
    memset(&event_data,                 0, sizeof(event_data_t              ));
    memset(&referee_warning,            0, sizeof(referee_warning_t         ));
    memset(&dart_info,                  0, sizeof(dart_info_t               ));
    memset(&robot_status,               0, sizeof(robot_status_t            ));
    memset(&power_heat_data,            0, sizeof(power_heat_data_t         ));
    memset(&robot_pos,                  0, sizeof(robot_pos_t               ));
    memset(&buff,                       0, sizeof(buff_t                  	));
    memset(&hurt_data,                  0, sizeof(hurt_data_t               ));
    memset(&shoot_data,                 0, sizeof(shoot_data_t              ));
    memset(&projectile_allowance,       0, sizeof(projectile_allowance_t    ));
    memset(&rfid_status,                0, sizeof(rfid_status_t             ));
    memset(&dart_client_cmd,            0, sizeof(dart_client_cmd_t         ));
    memset(&ground_robot_position,      0, sizeof(ground_robot_position_t   ));
    memset(&radar_mark_data,            0, sizeof(radar_mark_data_t         ));
    memset(&sentry_info,                0, sizeof(sentry_info_t             ));
    memset(&radar_info,                 0, sizeof(radar_info_t              ));
    memset(&robot_interaction_data,     0, sizeof(robot_interaction_data_t  ));
    memset(&interaction_layer_delete,   0, sizeof(interaction_layer_delete_t));
    memset(&interaction_figure,         0, sizeof(interaction_figure_t      ));
    memset(&interaction_figure_2,       0, sizeof(interaction_figure_2_t    ));
    memset(&interaction_figure_3,       0, sizeof(interaction_figure_3_t    ));
    memset(&interaction_figure_4,       0, sizeof(interaction_figure_4_t    ));
    memset(&client_custom_character,    0, sizeof(client_custom_character_t ));
    memset(&sentry_cmd,                 0, sizeof(sentry_cmd_t              ));
    memset(&radar_cmd,                  0, sizeof(radar_cmd_t             	));
    memset(&map_command,                0, sizeof(map_command_t             ));
    memset(&map_robot_data,             0, sizeof(map_robot_data_t          ));
    memset(&map_data,                   0, sizeof(map_data_t                ));
    memset(&custom_info,                0, sizeof(custom_info_t             ));
    memset(&custom_robot_data,          0, sizeof(custom_robot_data_t       ));
    memset(&robot_custom_data,          0, sizeof(robot_custom_data_t       ));
    memset(&robot_custom_data_2,        0, sizeof(robot_custom_data_2_t     ));
    memset(&robot_custom_data_3,        0, sizeof(robot_custom_data_3_t     ));
    memset(&custom_client_data,         0, sizeof(custom_client_data_t      ));
    memset(&enemy_pos,                  0, sizeof(enemy_pos_t               ));
    memset(&enemy_hp,                   0, sizeof(enemy_hp_t              	));
    memset(&enemy_ammo,                 0, sizeof(enemy_ammo_t              ));
    memset(&enemy_macro_status,         0, sizeof(enemy_macro_status_t			));
    memset(&enemy_buff,                 0, sizeof(enemy_buff_t              ));
    memset(&enemy_wave_key,             0, sizeof(enemy_wave_key_t          ));
    g_referee_last_rx_ms = 0;
}

/**
  * @brief          兼容旧接口的裁判数据初始化入口
  * @retval         none
  */
void init_referee_struct_data(void)
{
    init_referee_data();
}

/**
  * @brief          按 cmd_id 分发裁判系统数据帧
  * @param[in]      frame: 裁判协议完整帧指针
  * @note           成功解析后刷新 g_referee_last_rx_ms，供在线状态判断使用。
  * @retval         none
  */
void referee_handle_data(uint8_t *frame)
{
	uint16_t cmd_id = 0;
	uint8_t index = 0;
	g_referee_last_rx_ms = HAL_GetTick();
	
	//下面这个size_of_header_struct_t的值是6, 因为没有强制按1字节对齐, 此函数后面的5都是直接赋值了
	//uint8_t size_of_header_struct_t = sizeof(frame_header_struct_t);
	//memcpy(&referee_receive_header, frame, size_of_header_struct_t);
	
	memcpy(&referee_receive_header, frame, 5);

	index += 5;

	memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	index += sizeof(uint16_t);

	switch (cmd_id)
	{
		case GAME_STATUS_CMD_ID             : { memcpy(&game_status,							frame + index, sizeof(game_status_t             ));} break;
		case GAME_RESULT_CMD_ID             : { memcpy(&game_result,              frame + index, sizeof(game_result_t             ));} break;
		case GAME_ROBOT_HP_CMD_ID           : { memcpy(&game_robot_HP,            frame + index, sizeof(game_robot_HP_t           ));} break;
		case EVENT_DATA_CMD_ID              : { memcpy(&event_data,               frame + index, sizeof(event_data_t              ));} break;
		case REFEREE_WARNING_CMD_ID         : { memcpy(&referee_warning,          frame + index, sizeof(referee_warning_t         ));} break;
		case DART_INFO_CMD_ID               : { memcpy(&dart_info,                frame + index, sizeof(dart_info_t               ));} break;
		case ROBOT_STATUS_CMD_ID            : { memcpy(&robot_status,             frame + index, sizeof(robot_status_t            ));} break;
		case POWER_HEAT_DATA_CMD_ID         : { memcpy(&power_heat_data,          frame + index, sizeof(power_heat_data_t         ));} break;
		case ROBOT_POS_CMD_ID               : { memcpy(&robot_pos,                frame + index, sizeof(robot_pos_t               ));} break;
		case BUFF_CMD_ID                    : { memcpy(&buff,                     frame + index, sizeof(buff_t                  	));} break;
		case HURT_DATA_CMD_ID               : { memcpy(&hurt_data,                frame + index, sizeof(hurt_data_t               ));} break;
		case SHOOT_DATA_CMD_ID              : { memcpy(&shoot_data,               frame + index, sizeof(shoot_data_t              ));} break;
		case PROJECTILE_ALLOWANCE_CMD_ID    : { memcpy(&projectile_allowance,     frame + index, sizeof(projectile_allowance_t    ));} break;
		case RFID_STATUS_CMD_ID             : { memcpy(&rfid_status,              frame + index, sizeof(rfid_status_t             ));} break;
		case DART_CLIENT_CMD_CMD_ID         : { memcpy(&dart_client_cmd,          frame + index, sizeof(dart_client_cmd_t         ));} break;
		case GROUND_ROBOT_POSITION_CMD_ID   : { memcpy(&ground_robot_position,    frame + index, sizeof(ground_robot_position_t   ));} break;
		case RADAR_MARK_DATA_CMD_ID         : { memcpy(&radar_mark_data,          frame + index, sizeof(radar_mark_data_t         ));} break;
		case SENTRY_INFO_CMD_ID             : { memcpy(&sentry_info,              frame + index, sizeof(sentry_info_t             ));} break;
		case RADAR_INFO_CMD_ID              : { memcpy(&radar_info,               frame + index, sizeof(radar_info_t              ));} break;
		case ROBOT_INTERACTION_DATA_CMD_ID  : { memcpy(&robot_interaction_data,   frame + index, sizeof(robot_interaction_data_t  ));} break;
//		case INTERACTION_LAYER_DELETE_CMD_ID: { memcpy(&interaction_layer_delete, frame + index, sizeof(interaction_layer_delete_t));} break;
//		case INTERACTION_FIGURE_CMD_ID      : { memcpy(&interaction_figure,       frame + index, sizeof(interaction_figure_t      ));} break;
//		case INTERACTION_FIGURE_2_CMD_ID    : { memcpy(&interaction_figure_2,     frame + index, sizeof(interaction_figure_2_t    ));} break;
//		case INTERACTION_FIGURE_3_CMD_ID    : { memcpy(&interaction_figure_3,     frame + index, sizeof(interaction_figure_3_t    ));} break;
//		case INTERACTION_FIGURE_4_CMD_ID    : { memcpy(&interaction_figure_4,     frame + index, sizeof(interaction_figure_4_t    ));} break;
		case CLIENT_CUSTOM_CHARACTER_CMD_ID : { memcpy(&client_custom_character,  frame + index, sizeof(client_custom_character_t ));} break;
		case SENTRY_CMD_CMD_ID              : { memcpy(&sentry_cmd,               frame + index, sizeof(sentry_cmd_t              ));} break;
		case RADAR_CMD_CMD_ID               : { memcpy(&radar_cmd,                frame + index, sizeof(radar_cmd_t             	));} break;
		case MAP_COMMAND_CMD_ID             : { memcpy(&map_command,              frame + index, sizeof(map_command_t             ));} break;
		case MAP_ROBOT_DATA_CMD_ID          : { memcpy(&map_robot_data,           frame + index, sizeof(map_robot_data_t          ));} break;
		case MAP_DATA_CMD_ID                : { memcpy(&map_data,                 frame + index, sizeof(map_data_t                ));} break;
		case CUSTOM_INFO_CMD_ID             : { memcpy(&custom_info,              frame + index, sizeof(custom_info_t             ));} break;
		case CUSTOM_ROBOT_DATA_CMD_ID       : { memcpy(&custom_robot_data,        frame + index, sizeof(custom_robot_data_t       ));} break;
		case ROBOT_CUSTOM_DATA_CMD_ID       : { memcpy(&robot_custom_data,        frame + index, sizeof(robot_custom_data_t       ));} break;
		case ROBOT_CUSTOM_DATA_2_CMD_ID     : { memcpy(&robot_custom_data_2,      frame + index, sizeof(robot_custom_data_2_t     ));} break;
		case ROBOT_CUSTOM_DATA_3_CMD_ID     : { memcpy(&robot_custom_data_3,      frame + index, sizeof(robot_custom_data_3_t     ));} break;
		case CUSTOM_CLIENT_DATA_CMD_ID      : { memcpy(&custom_client_data,       frame + index, sizeof(custom_client_data_t      ));} break;
		case ENEMY_POS_CMD_ID               : { memcpy(&enemy_pos,                frame + index, sizeof(enemy_pos_t               ));} break;
		case ENEMY_HP_CMD_ID                : { memcpy(&enemy_hp,                 frame + index, sizeof(enemy_hp_t              	));} break;
		case ENEMY_AMMO_CMD_ID              : { memcpy(&enemy_ammo,               frame + index, sizeof(enemy_ammo_t              ));} break;
		case ENEMY_MACRO_STATUS_CMD_ID      : { memcpy(&enemy_macro_status,       frame + index, sizeof(enemy_macro_status_t			));} break;
		case ENEMY_BUFF_CMD_ID              : { memcpy(&enemy_buff,               frame + index, sizeof(enemy_buff_t              ));} break;
		case ENEMY_WAVE_KEY_CMD_ID          : { memcpy(&enemy_wave_key,           frame + index, sizeof(enemy_wave_key_t          ));} break;

		default: { break; }
	}
}

/**
  * @brief          裁判数据解析兼容入口
  * @param[in]      frame: 裁判协议完整帧指针
  * @retval         none
  */
void referee_data_solve(uint8_t *frame)
{
	referee_handle_data(frame);
}

//获取机器人ID
uint8_t get_robot_id(void)
{
	return robot_status.robot_id;
}

//获取射击热量上限
uint16_t get_shooter_barrel_heat_limit(void)
{
	return robot_status.shooter_barrel_heat_limit;
}

//获取底盘功率上限
uint16_t get_chassis_power_limit(void)
{
	return robot_status.chassis_power_limit;
}

//获取缓冲能量
uint16_t get_buffer_energy(void)
{
	return power_heat_data.buffer_energy;
}

//获取17mm枪口热量
uint16_t get_shooter_17mm_heat(void)
{
	return power_heat_data.shooter_17mm_barrel_heat;
}

//获取42mm枪口热量
uint16_t get_shooter_42mm_heat(void)
{
	return power_heat_data.shooter_42mm_barrel_heat;
}

uint8_t referee_data_available(uint32_t timeout_ms)
{
	uint32_t now = HAL_GetTick();
	uint8_t robot_id = robot_status.robot_id;

	if((robot_id >= RED_HERO && robot_id <= RED_SENTRY) || (robot_id >= BLUE_HERO && robot_id <= BLUE_SENTRY))
	{
		return (uint8_t)((now - g_referee_last_rx_ms) <= timeout_ms);
	}

	return 0;
}

void get_chassis_power_and_buffer(uint16_t *power_limit, uint16_t *buffer)
{
	if(power_limit != NULL)
	{
		*power_limit = robot_status.chassis_power_limit;
	}

	if(buffer != NULL)
	{
		*buffer = power_heat_data.buffer_energy;
	}
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat_limit, uint16_t *heat)
{
	if(heat_limit != NULL)
	{
		*heat_limit = robot_status.shooter_barrel_heat_limit;
	}

	if(heat != NULL)
	{
		*heat = power_heat_data.shooter_17mm_barrel_heat;
	}
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat_limit, uint16_t *heat)
{
	if(heat_limit != NULL)
	{
		*heat_limit = robot_status.shooter_barrel_heat_limit;
	}

	if(heat != NULL)
	{
		*heat = power_heat_data.shooter_42mm_barrel_heat;
	}
}
