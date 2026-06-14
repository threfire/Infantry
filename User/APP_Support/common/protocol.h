/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       protocol.h
  * @brief     	裁判系统通信协议
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

#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#ifndef REF_PROTOCOL_FRAME_MAX_SIZE
#define REF_PROTOCOL_FRAME_MAX_SIZE         192
#endif

#define REF_PROTOCOL_HEADER_SIZE            5
#define REF_PROTOCOL_CMDID_SIZE             2
#define REF_PROTOCOL_CMD_SIZE               REF_PROTOCOL_CMDID_SIZE
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CMD_SIZE + REF_PROTOCOL_CRC16_SIZE)

//帧头格式
typedef  struct
{
  uint8_t SOF;							//数据帧起始字节，固定值为0xA5
  uint16_t data_length;			//数据帧中data的长度
  uint8_t seq;							//包序号
  uint8_t CRC8;							//帧头CRC8校验
} frame_header_struct_t;

//解包步骤枚举
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

//数据解包
typedef struct
{
  frame_header_struct_t		*p_header;
  uint16_t       					data_length;
  uint8_t									protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e						unpack_step;
  uint16_t								index;
} unpack_data_t;

//机器人ID
typedef enum
{
	RED_HERO        = 1,
	RED_ENGINEER    = 2,
	RED_INFANTRY_1  = 3,
	RED_INFANTRY_2  = 4,
	RED_INFANTRY_3  = 5,
	RED_AERIAL      = 6,
	RED_SENTRY      = 7,
	RED_DART				= 8,
	RED_RADAR				= 9,
	RED_OUTPOST			= 10,
	RED_BASE				= 11,
	BLUE_HERO       = 101,
	BLUE_ENGINEER   = 102,
	BLUE_INFANTRY_1 = 103,
	BLUE_INFANTRY_2 = 104,
	BLUE_INFANTRY_3 = 105,
	BLUE_AERIAL     = 106,
	BLUE_SENTRY     = 107,
	BLUE_DART				= 108,
	BLUE_RADAR			= 109,
	BLUE_OUTPOST		= 110,
	BLUE_BASE				= 111,
}robot_id_t;

//命令码ID
typedef enum
{
	GAME_STATUS_CMD_ID = 0x0001,                	/* 游戏状态命令ID */
	GAME_RESULT_CMD_ID = 0x0002,                	/* 游戏结果命令ID */
	GAME_ROBOT_HP_CMD_ID = 0x0003,              	/* 游戏机器人血量命令ID */
	EVENT_DATA_CMD_ID = 0x0101,           				/* 事件数据命令ID */
	REFEREE_WARNING_CMD_ID = 0x0104,          		/* 裁判警告命令ID */
	DART_INFO_CMD_ID = 0x0105,                    /* 飞镖信息命令ID */
	ROBOT_STATUS_CMD_ID = 0x0201,                 /* 机器人状态命令ID */
	POWER_HEAT_DATA_CMD_ID = 0x0202,             	/* 功率热量数据命令ID */
	ROBOT_POS_CMD_ID = 0x0203,                  	/* 机器人位置命令ID */
	BUFF_CMD_ID = 0x0204,                      		/* 增益效果命令ID */
	HURT_DATA_CMD_ID = 0x0206,                   	/* 伤害数据命令ID */
	SHOOT_DATA_CMD_ID = 0x0207,                  	/* 射击数据命令ID */
	PROJECTILE_ALLOWANCE_CMD_ID = 0x0208,        	/* 弹丸余量命令ID */
	RFID_STATUS_CMD_ID = 0x0209,                 	/* RFID状态命令ID */
	DART_CLIENT_CMD_CMD_ID = 0x020A,            	/* 飞镖客户端命令命令ID */
	GROUND_ROBOT_POSITION_CMD_ID = 0x020B,      	/* 地面机器人位置命令ID */
	RADAR_MARK_DATA_CMD_ID = 0x020C,             	/* 雷达标记数据命令ID */
	SENTRY_INFO_CMD_ID = 0x020D,                 	/* 哨兵信息命令ID */
	RADAR_INFO_CMD_ID = 0x020E,                  	/* 雷达信息命令ID */
	
	ROBOT_INTERACTION_DATA_CMD_ID = 0x0301,    		/* 机器人交互数据命令ID */
	//下面的ID为0x0301的子ID
	INTERACTION_LAYER_DELETE_CMD_ID = 0x0100,  		/* 交互层删除命令ID */
	INTERACTION_FIGURE_CMD_ID = 0x0101,        		/* 交互图形命令ID */
	INTERACTION_FIGURE_2_CMD_ID = 0x0102,      		/* 交互图形2命令ID */
	INTERACTION_FIGURE_3_CMD_ID = 0x0103,      		/* 交互图形3命令ID */
	INTERACTION_FIGURE_4_CMD_ID = 0x0104,      		/* 交互图形4命令ID */
	CLIENT_CUSTOM_CHARACTER_CMD_ID = 0x0110,   		/* 客户端自定义字符命令ID */
	SENTRY_CMD_CMD_ID = 0x0120,                		/* 哨兵命令命令ID */
	RADAR_CMD_CMD_ID = 0x0121,                 		/* 雷达命令命令ID */
	
	MAP_COMMAND_CMD_ID = 0x0303,           				/* 地图命令命令ID */
	MAP_ROBOT_DATA_CMD_ID = 0x0305,        				/* 地图机器人数据命令ID */
	MAP_DATA_CMD_ID = 0x0307,              				/* 地图数据命令ID */
	CUSTOM_INFO_CMD_ID = 0x0308,            			/* 自定义信息命令ID */
	CUSTOM_ROBOT_DATA_CMD_ID = 0x0302,      			/* 自定义机器人数据命令ID */
	ROBOT_CUSTOM_DATA_CMD_ID = 0x0309,      			/* 机器人自定义数据命令ID */
	ROBOT_CUSTOM_DATA_2_CMD_ID = 0x0310,    			/* 机器人自定义数据2命令ID */
	ROBOT_CUSTOM_DATA_3_CMD_ID = 0x0311,    			/* 机器人自定义数据3命令ID */
	CUSTOM_CLIENT_DATA_CMD_ID = 0x0306,     			/* 自定义客户端数据命令ID */
	
	/* 雷达无线链路 */
	ENEMY_POS_CMD_ID = 0x0A01,           					/* 对方机器人位置命令ID */
	ENEMY_HP_CMD_ID = 0x0A02,            					/* 对方机器人血量命令ID */
	ENEMY_AMMO_CMD_ID = 0x0A03,          					/* 对方剩余发弹量命令ID */
	ENEMY_MACRO_STATUS_CMD_ID = 0x0A04,  					/* 对方宏观状态命令ID */
	ENEMY_BUFF_CMD_ID = 0x0A05,          					/* 对方增益效果命令ID */
	ENEMY_WAVE_KEY_CMD_ID = 0x0A06,      					/* 对方干扰波密钥命令ID */
} referee_cmd_id_t;

//对应0x0001中的game_progress
typedef enum
{
	PROGRESS_UNSTART        = 0,
	PROGRESS_PREPARE        = 1,
	PROGRESS_SELFCHECK      = 2,
	PROGRESS_5sCOUNTDOWN    = 3,
	PROGRESS_BATTLE         = 4,
	PROGRESS_CALCULATING    = 5,
}game_progress_t;


/* 命令码对应数据结构 */
#pragma pack(push, 1)

//0x0001
typedef struct { uint8_t game_type : 4; uint8_t game_progress : 4; uint16_t stage_remain_time; uint64_t SyncTimeStamp; }game_status_t;
//0x0002
typedef struct { uint8_t winner; }game_result_t;
//0x0003
typedef struct { uint16_t ally_1_robot_HP; uint16_t ally_2_robot_HP; uint16_t ally_3_robot_HP; uint16_t ally_4_robot_HP;
								 uint16_t reserved; uint16_t ally_7_robot_HP; uint16_t ally_outpost_HP; uint16_t ally_base_HP; }game_robot_HP_t;
//0x0101
typedef struct { uint32_t event_data; }event_data_t;
//0x0104
typedef struct { uint8_t level; uint8_t offending_robot_id; uint8_t count; }referee_warning_t;
//0x0105
typedef struct { uint8_t dart_remaining_time; uint16_t dart_info; }dart_info_t;
//0x0201
typedef struct { uint8_t robot_id; uint8_t robot_level; uint16_t current_HP; uint16_t maximum_HP; uint16_t shooter_barrel_cooling_value; uint16_t shooter_barrel_heat_limit;
								 uint16_t chassis_power_limit; uint8_t power_management_gimbal_output : 1; uint8_t power_management_chassis_output : 1; uint8_t power_management_shooter_output : 1; }robot_status_t;
//0x0202
typedef struct { uint16_t reserved_0; uint16_t reserved_1; float reserved_2; uint16_t buffer_energy; uint16_t shooter_17mm_barrel_heat; uint16_t shooter_42mm_barrel_heat; }power_heat_data_t;
//0x0203
typedef struct { float x; float y; float angle; }robot_pos_t;
//0x0204
typedef struct { uint8_t recovery_buff; uint16_t cooling_buff; uint8_t defence_buff; uint8_t vulnerability_buff; uint16_t attack_buff; uint8_t remaining_energy; }buff_t;
//0x0206
typedef struct { uint8_t armor_id : 4; uint8_t HP_deduction_reason : 4; }hurt_data_t;
//0x0207
typedef struct { uint8_t bullet_type; uint8_t shooter_number; uint8_t launching_frequency; float initial_speed; }shoot_data_t;
//0x0208
typedef struct { uint16_t projectile_allowance_17mm; uint16_t projectile_allowance_42mm; uint16_t remaining_gold_coin; uint16_t projectile_allowance_fortress; }projectile_allowance_t;
//0x0209
typedef struct { uint32_t rfid_status; uint8_t rfid_status_2; }rfid_status_t;
//0x020A
typedef struct { uint8_t dart_launch_opening_status; uint8_t reserved; uint16_t target_change_time; uint16_t latest_launch_cmd_time; }dart_client_cmd_t;
//0x020B
typedef struct { float hero_x; float hero_y; float engineer_x; float engineer_y; float standard_3_x; float standard_3_y; float standard_4_x; float standard_4_y; float reserved_0; float reserved_1; }ground_robot_position_t;
//0x020C
typedef struct { uint16_t mark_progress; }radar_mark_data_t;
//0x020D
typedef struct { uint32_t sentry_info; uint16_t sentry_info_2; }sentry_info_t;
//0x020E
typedef struct { uint8_t radar_info; }radar_info_t;

//0x0301
typedef struct { uint16_t data_cmd_id; uint16_t sender_id; uint16_t receiver_id; uint8_t user_data[112]; }robot_interaction_data_t;
//0x0100
typedef struct { uint8_t delete_type; uint8_t layer; }interaction_layer_delete_t;
//0x0101
typedef struct { uint8_t figure_name[3]; uint32_t operate_tpye:3; uint32_t figure_tpye:3; uint32_t layer:4; uint32_t color:4; uint32_t details_a:9; uint32_t details_b:9;
								 uint32_t width:10; uint32_t start_x:11; uint32_t start_y:11; uint32_t details_c:10; uint32_t details_d:11; uint32_t details_e:11; }interaction_figure_t;
//0x0102
typedef struct { interaction_figure_t interaction_figure[2]; }interaction_figure_2_t;
//0x0103
typedef struct { interaction_figure_t interaction_figure[5]; }interaction_figure_3_t;
//0x0104
typedef struct { interaction_figure_t interaction_figure[7]; }interaction_figure_4_t;
//0x0110
typedef struct { uint8_t data[30]; }client_custom_character_t;
//0x0120
typedef struct { uint32_t sentry_cmd; }sentry_cmd_t;
//0x0121
typedef struct { uint8_t radar_cmd; uint8_t password_cmd; uint8_t password_1; uint8_t password_2; uint8_t password_3; uint8_t password_4; uint8_t password_5; uint8_t password_6; }radar_cmd_t;

//0x0303
typedef struct { float target_position_x; float target_position_y; uint8_t cmd_keyboard; uint8_t target_robot_id; uint16_t cmd_source; }map_command_t;
//0x0305
typedef struct { uint16_t hero_position_x; uint16_t hero_position_y; uint16_t engineer_position_x; uint16_t engineer_position_y; uint16_t infantry_3_position_x; uint16_t infantry_3_position_y;
								 uint16_t infantry_4_position_x; uint16_t infantry_4_position_y; uint16_t reserved_0; uint16_t reserved_1; uint16_t sentry_position_x; uint16_t sentry_position_y; }map_robot_data_t;
//0x0307
typedef struct { uint8_t intention; uint16_t start_position_x; uint16_t start_position_y; int8_t delta_x[49]; int8_t delta_y[49]; uint16_t sender_id; }map_data_t;
//0x0308
typedef struct { uint16_t sender_id; uint16_t receiver_id; uint8_t user_data[30]; }custom_info_t;
//0x0302
typedef struct { uint8_t data[30]; }custom_robot_data_t;
//0x0309
typedef struct { uint8_t data[30]; }robot_custom_data_t;
//0x0310
typedef struct { uint8_t data[300]; }robot_custom_data_2_t;
//0x0311
typedef struct { uint8_t data[30]; }robot_custom_data_3_t;
//0x0306
typedef struct { uint16_t key_value; uint16_t x_position:12; uint16_t mouse_left:4; uint16_t y_position:12; uint16_t mouse_right:4; uint16_t reserved; }custom_client_data_t;

// 0x0A01
typedef struct { uint16_t hero_x, hero_y; uint16_t engineer_x, engineer_y; uint16_t infantry3_x, infantry3_y; uint16_t infantry4_x, infantry4_y; uint16_t aerial_x, aerial_y; uint16_t sentry_x, sentry_y; }enemy_pos_t;
// 0x0A02
typedef struct { uint16_t hero1_hp; uint16_t engineer2_hp; uint16_t infantry3_hp; uint16_t infantry4_hp; uint16_t reserved; uint16_t sentry7_hp; }enemy_hp_t;
// 0x0A03
typedef struct { uint16_t hero1_ammo; uint16_t infantry3_ammo; uint16_t infantry4_ammo; uint16_t aerial6_ammo; uint16_t sentry7_ammo; }enemy_ammo_t;
// 0x0A04
typedef struct { uint16_t remaining_gold; uint16_t total_gold; uint32_t occupation_status; }enemy_macro_status_t;
// 0x0A05
typedef struct { uint8_t hero_recovery; uint16_t hero_cooling; uint8_t hero_defence; uint8_t hero_vulnerability; uint16_t hero_attack; uint8_t engineer_recovery; uint16_t engineer_cooling; uint8_t engineer_defence;
								 uint8_t engineer_vulnerability; uint16_t engineer_attack; uint8_t infantry3_recovery; uint16_t infantry3_cooling; uint8_t infantry3_defence; uint8_t infantry3_vulnerability; uint16_t infantry3_attack;
								 uint8_t infantry4_recovery; uint16_t infantry4_cooling; uint8_t infantry4_defence; uint8_t infantry4_vulnerability; uint16_t infantry4_attack; uint8_t sentry_recovery; uint16_t sentry_cooling;
								 uint8_t sentry_defence; uint8_t sentry_vulnerability; uint16_t sentry_attack; uint8_t sentry_posture; }enemy_buff_t;
// 0x0A06
typedef struct { uint8_t key[6]; }enemy_wave_key_t;

#pragma pack(pop)

#endif
