/**
  * @file       chassis_power_control.c
  * @brief      底盘功率预测与电流限制
  * @note       根据电机模型和功率上限缩放底盘电流命令。
  */

/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c
  * @brief      ??????????
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
/**
  * @file       chassis_power_control.c
  * @brief      底盘功率控制实现文件
  * @details    实现基于功率预测的底盘电机电流限制逻辑，通过计算电机功率消耗�?
  *             对比额定功率阈值，动态调整电机输出电流，防止功率超限
  * @author     用户自定�?
  * @date       2026-1-9
  */
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "referee.h"
#include <math.h>
#include <stdbool.h>
#include "pm01_api.h"

// 外部变量声明（来自其他任务模块）
extern RC_ctrl_t rc_ctrl;                		// 遥控�?键鼠控制输入
extern robot_status_t robot_status;      		// 机器人状态（裁判系统数据�?
extern power_heat_data_t power_heat_data;		// 缓冲能量数据（裁判系统）
extern chassis_move_t chassis_move;      		// 底盘运动控制结构体（含电机PID、电机测量数据）

// 自定义变�?
super_cap_mode_e super_cap_mode = SUPER_CAP_CHARGING;

// 全局功率限制控制结构体初始化
PowerLimit_t PowerLimit = {
	.k_1 = 1.453e-07f,  										// 转速平方项系数（功率计算模型）
	.k_2 = 1.23e-07f,   										// 电流平方项系数（功率计算模型�?
	.a = 4.081f,        										// 功率模型基础偏移�?
	.K_Reduction = 1.0f,										// 功率衰减系数（初始为1，无衰减�?
	.set_power = SET_POWER_VALUE,		// 额定功率设定值（来自头文件宏定义�?
	.P_origin = 0.0f,												// 原始功率计算值（未衰减前�?
	.P_bus = 0.0f    												// 3508衰减后功率（衰减后）
};

/* 原始功率计算公式说明�?
 * P_origin = a + ω*b*I_cmd + k1*ω² + k2*b²*I_cmd² 
 * 其中：b为衰减系数（初始�?），ω为电机转速，I_cmd为目标电�?
 */

/**
 * @brief  电流约束关系计算函数
 * @param  PowerLimit_Cur 功率限制结构体指�?
 * @note   从底盘运动结构体中读�?508/6020电机的实时状态数据，
 *         填充到功率限制结构体中，为后续功率计算提供基础数据
 */
/**
  * @brief          计算底盘电流限幅关系
  * @param[out]     PowerLimit_Cur: 功率限制状态结构体指针
  * @retval         none
  */
void Current_RestraintRelation_Calc(PowerLimit_t *PowerLimit_Cur)
{
	// 遍历4个电机通道（底盘通常�?�?508电机�?
	for(int i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		// 获取3508电机实时数据
		// 3508电机当前实际转速（rpm），来自电机反馈测量�?
		PowerLimit_Cur->now_motorspeed[i] = chassis_move.chassis_3508[i].chassis_motor_measure->speed_rpm;
		// 3508���Ŀ���������ֵ��cmd�������Ե�������ģ�����
		PowerLimit_Cur->set_motorcurrent[i] = chassis_move.model_3508_out[i];
		
		// 获取6020电机实时数据（修正字段名与结构体匹配�?
		// 6020电机当前实际转速（rpm�?
		// 6020电机目标电流（A），来自速度环PID输出
		// 6020电机实际发送的电流值（A�?
	}
}
static float chassis_motor_power_calc(const PowerLimit_t *PowerLimit_Pre, float omega, float current_cmd)
{
	const float current_power_coeff = (0.01562f * 0.001220703125f) / 9.55f;

	return PowerLimit_Pre->a +
	       current_power_coeff * omega * current_cmd +
	       PowerLimit_Pre->k_1 * omega * omega +
	       PowerLimit_Pre->k_2 * current_cmd * current_cmd;
}

static float chassis_select_power_solution(float i_cmd, float root_a, float root_b)
{
	if(i_cmd > 0.0f)
	{
		if(root_a >= 0.0f && root_b >= 0.0f)
		{
			return (fabsf(root_a - i_cmd) < fabsf(root_b - i_cmd)) ? root_a : root_b;
		}
		if(root_a >= 0.0f)
		{
			return root_a;
		}
		if(root_b >= 0.0f)
		{
			return root_b;
		}
		return 0.0f;
	}

	if(i_cmd < 0.0f)
	{
		if(root_a <= 0.0f && root_b <= 0.0f)
		{
			return (fabsf(root_a - i_cmd) < fabsf(root_b - i_cmd)) ? root_a : root_b;
		}
		if(root_a <= 0.0f)
		{
			return root_a;
		}
		if(root_b <= 0.0f)
		{
			return root_b;
		}
		return 0.0f;
	}

	return 0.0f;
}

static float chassis_current_from_power(const PowerLimit_t *PowerLimit_Pre,
                                        float omega,
                                        float i_cmd,
                                        float target_power)
{
	const float current_power_coeff = (0.01562f * 0.001220703125f) / 9.55f;
	const float A = PowerLimit_Pre->k_2;
	const float B = current_power_coeff * omega;
	const float C = PowerLimit_Pre->a + PowerLimit_Pre->k_1 * omega * omega - target_power;
	const float discriminant = B * B - 4.0f * A * C;
	float sqrt_disc;
	float root_a;
	float root_b;

	if(discriminant < 0.0f || A == 0.0f)
	{
		return 0.0f;
	}

	sqrt_disc = sqrtf(discriminant);
	root_a = (-B + sqrt_disc) / (2.0f * A);
	root_b = (-B - sqrt_disc) / (2.0f * A);

	return chassis_select_power_solution(i_cmd, root_a, root_b);
}

/**
 * @brief  功率预测与电流衰减控制函�?
 * @param  PowerLimit_Pre 功率限制结构体指�?
 * @param  chassis_move   底盘运动控制结构体指�?
 * @note   核心功率控制逻辑�?
 *         1. 计算3508电机的理论功率消�?
 *         2. 对比额定功率，若超限则求解衰减系数b
 *         3. 应用衰减系数�?508电机电流，限制总功�?
 *         4. 6020电机不做衰减，保留预留功�?
 */
/**
  * @brief          预测底盘功率并缩放电流
  * @param[in,out]  PowerLimit_Pre: 功率限制状态结构体指针
  * @param[in,out]  chassis_move: 底盘控制结构体指针
  * @retval         none
  */
void Predict_Power(PowerLimit_t *PowerLimit_Pre, chassis_move_t *chassis_move)
{
	// 6020电机预留功率（W），保证云台/其他关节电机基础功�?
	// 3508底盘电机可用功率 = 总额定功�?- 6020预留功率
	const float available_power = PowerLimit_Pre->set_power;
	float motor_power[CHASSIS_MODULE_NUM] = {0.0f};
	float power_scale = 1.0f;

	PowerLimit.P_origin = 0.0f;
	PowerLimit.P_bus = 0.0f;

	for(int i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		const float omega = PowerLimit_Pre->now_motorspeed[i];
		const float i_cmd = PowerLimit_Pre->set_motorcurrent[i];

		motor_power[i] = chassis_motor_power_calc(PowerLimit_Pre, omega, i_cmd);
		PowerLimit.P_origin += motor_power[i];
	}

	if(PowerLimit.P_origin > available_power && PowerLimit.P_origin > 0.0f)
	{
		power_scale = available_power / PowerLimit.P_origin;
		power_scale = LIMIT_MAX_MIN(power_scale, 1.0f, 0.01f);
	}

	PowerLimit_Pre->K_Reduction = power_scale;

	for(int i = 0; i < CHASSIS_MODULE_NUM; i++)
	{
		const float omega = PowerLimit_Pre->now_motorspeed[i];
		const float i_cmd = PowerLimit_Pre->set_motorcurrent[i];
		float final_current = i_cmd;

		if(power_scale < 1.0f && motor_power[i] > 0.0f)
		{
			final_current = chassis_current_from_power(PowerLimit_Pre,
			                                          omega,
			                                          i_cmd,
			                                          motor_power[i] * power_scale);
		}

		chassis_move->chassis_3508[i].give_current = (int16_t)final_current;
		PowerLimit.P_bus += chassis_motor_power_calc(PowerLimit_Pre, omega, final_current);
	}


}
		
		
		
extern uint16_t g_cmd_set;

/**
 * @brief  底盘功率控制主函�?
 * @param  chassis_move 底盘运动控制结构体指�?
 * @note   底盘功率控制入口函数，供底盘任务主循环调用：
 *         1. 读取电机实时数据
 *         2. 计算功率并动态调整电机电�?
 *         3. 可扩展对接裁判系统实时功率限制（当前注释掉）
 */
/**
  * @brief          底盘功率控制入口
  * @param[in,out]  chassis_move: 底盘控制结构体指针
  * @note           在底盘电流发送前执行，限制预测功率并更新最终电流。
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_move)
{
	
	pm01_access_poll();
	//3V3对抗赛、步兵对抗赛时设置底盘最大功�?
	static bool have_set_power = false;
	if(robot_status.chassis_power_limit != SET_POWER_VALUE && robot_status.chassis_power_limit != 0 && !have_set_power)
	{
		have_set_power = true;
		PowerLimit.set_power = robot_status.chassis_power_limit * 0.8f;
	}
	
	Current_RestraintRelation_Calc(&PowerLimit);
	Predict_Power(&PowerLimit, chassis_move);
}
