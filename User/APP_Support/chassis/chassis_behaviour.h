/**
  * @file       chassis_behaviour.c
  * @brief      底盘行为模式设置
  * @note       
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

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

#include "chassis_task.h"
#include "struct_typedef.h"

typedef enum
{
	CHASSIS_NO_MOVE,								//底盘保持静止
	CHASSIS_FOLLOW_GIMBAL_YAW,			//底盘跟随云台yaw角度
	CHASSIS_YAW_HOLD,							//底盘yaw目标角闭环
	CHASSIS_SPIN,										//旋转模式
	CHASSIS_RETURN,									//底盘回正
	
} chassis_behaviour_e;

#define CHASSIS_OPEN_RC_SCALE 10		//在 CHASSIS_OPEN 模式下, 遥控器乘以该比例发送到can上


//根据遥控器输入设置底盘行为模式
/**
  * @brief          设置底盘行为模式
  * @retval         none
  */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
//根据当前行为模式设置底盘控制参数
/**
  * @brief          根据底盘行为模式生成速度给定
  * @retval         none
  */
extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
/**
  * @brief          将遥控器输入转换为底盘平移速度
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

//全局变量，用于存储底盘行为模式
extern chassis_behaviour_e chassis_behaviour_mode;


#endif
