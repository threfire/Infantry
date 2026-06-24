/**
  * @file       robot_param.h
  * @brief      机器人全局参数宏
  * @note       集中定义机器人类型、CAN ID、云台、底盘、发射和功率控制参数。
  */
#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#ifdef __cplusplus
extern "C" {
#endif

#define PI 3.14159265358979323846f
#define GIMBAL_PI PI

/* ========================= 机器人全局配置 ========================= */
/* 代码运行模式 */
#define debug   0
#define release 1

/* 超级电容开关 */
#define Cap_off 0X00
#define Cap_on  0X01

#define ROBOT_MODE        debug     // 当前代码模式：debug/release
#define ROBOT_CAP         Cap_off   // 当前超级电容状态

#define ROBOT_TYPE_HERO                  1
#define ROBOT_GIMBAL_YAW_PITCH_DIRECT   1
#define ROBOT_CHASSIS_OMNI              1
#define ROBOT_FRICTION_3508             1

#define ROBOT_TYPE      ROBOT_TYPE_HERO
#define ROBOT_GIMBAL    ROBOT_GIMBAL_YAW_PITCH_DIRECT
#define ROBOT_CHASSIS   ROBOT_CHASSIS_OMNI
#define ROBOT_FRICTION  ROBOT_FRICTION_3508

/* ========================= 外部 Flash 日志配置 ========================= */
#define FLASH_LOG_ENABLE        0U   // 外部 Flash 错误日志开关，0=关闭，1=开启
#define FLASH_LOG_QUEUE_DEPTH   8U   // RAM 日志队列深度，控制循环只入队

/* ========================= 云台角度 PID 参数 ========================= */
/* pitch 轴陀螺仪绝对角控制 PID */
#define PITCH_GYRO_ABSOLUTE_PID_KP         4.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI         0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD         0.05f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT    1.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT   0.0f

/* yaw 轴陀螺仪绝对角控制 PID */
#define YAW_GYRO_ABSOLUTE_PID_KP           5.0f
#define YAW_GYRO_ABSOLUTE_PID_KI           0.01f
#define YAW_GYRO_ABSOLUTE_PID_KD           0.74f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT      2.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT     0.0f

/* pitch 轴编码器相对角控制 PID */
#define PITCH_ENCODE_RELATIVE_PID_KP       8.8f
#define PITCH_ENCODE_RELATIVE_PID_KI       0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD       0.70f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT  2.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

/* yaw 轴编码器相对角控制 PID */
#define YAW_ENCODE_RELATIVE_PID_KP         1.8f
#define YAW_ENCODE_RELATIVE_PID_KI         0.0f
#define YAW_ENCODE_RELATIVE_PID_KD         0.2f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT    0.8f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT   0.0f

/* ========================= 云台前馈与输出配置 ========================= */
#define YAW_REF_VEL_FILTER_ALPHA           0.05f   // yaw 目标角差分速度低通系数
#define YAW_REF_ACCEL_LIMIT                100.0f  // yaw 惯量前馈参考加速度限幅
#define PITCH_RELATIVE_SPEED_FILTER_ALPHA  0.20f   // pitch 编码器差分速度低通系数
#define PITCH_VELOCITY_FF_GAIN             0.1f    // pitch 速度前馈系数，单位 N*m/(rad/s)

/* 惯量前馈：torque_ff = J * alpha_ref */
#define YAW_INERTIA_KGM2                   0.013   // yaw 转动惯量 J，单位 kg*m^2
#define PITCH_EQ_MASS_KG                   1.5f    // pitch 重力补偿使用的等效质量
#define PITCH_INERTIA_KGM2                 0.00245 // pitch 转动惯量 J，单位 kg*m^2

/* ========================= 云台静摩擦补偿配置 ========================= */
#define GIMBAL_YAW_STATIC_FRICTION_COMP           0.28f
#define GIMBAL_YAW_STATIC_FRICTION_DEADBAND       0.002f
#define GIMBAL_YAW_STATIC_FRICTION_FULLBAND       0.006f
#define GIMBAL_YAW_STATIC_FRICTION_FILTER_ALPHA   0.2f

#define GIMBAL_PITCH_STATIC_FRICTION_COMP_UP      0.1f
#define GIMBAL_PITCH_STATIC_FRICTION_COMP_DOWN    0.00f
#define GIMBAL_PITCH_STATIC_FRICTION_DEADBAND     0.004f
#define GIMBAL_PITCH_STATIC_FRICTION_FULLBAND     0.008f
#define GIMBAL_PITCH_STATIC_FRICTION_FILTER_ALPHA 0.1f

/* ========================= 遥控与鼠标输入配置 ========================= */
#define GIMBAL_ANGLE_Z_RC_SEN              0.0000005f // 小陀螺底盘旋转角速度输入灵敏度
#define YAW_CHANNEL                        2          // yaw 遥控通道
#define PITCH_CHANNEL                      3          // pitch 遥控通道
#define GIMBAL_MODE_CHANNEL                0          // 云台模式切换通道
#define WZ_CHANNEL                         2          // 底盘旋转通道
#define TURN_KEYBOARD                      KEY_PRESSED_OFFSET_F // 小陀螺按键
#define TURN_SPEED                         0.04f      // 小陀螺旋转速度
#define TEST_KEYBOARD                      KEY_PRESSED_OFFSET_B // 测试按键
#define RC_DEADBAND                        10         // 遥控器死区
#define YAW_RC_SEN                         -0.000005f // yaw 遥控灵敏度
#define PITCH_RC_SEN                       -0.000006f // pitch 遥控灵敏度
#define YAW_MOUSE_SEN                      0.000006f  // yaw 鼠标灵敏度
#define PITCH_MOUSE_SEN                    -0.000006f // pitch 鼠标灵敏度
#define YAW_ENCODE_SEN                     0.01f      // yaw 编码器模式输入灵敏度
#define PITCH_ENCODE_SEN                   0.01f      // pitch 编码器模式输入灵敏度

/* ========================= 云台任务与反馈索引配置 ========================= */
#define GIMBAL_TASK_INIT_TIME              500    // 云台任务启动延时，单位 ms
#define GIMBAL_CONTROL_TIME                1      // 云台控制周期，单位 ms
#define INS_YAW_ADDRESS_OFFSET             0      // INS yaw 角数组索引
#define INS_PITCH_ADDRESS_OFFSET           1      // INS pitch 角数组索引
#define INS_ROLL_ADDRESS_OFFSET            2      // INS roll 角数组索引
#define INS_GYRO_X_ADDRESS_OFFSET          0      // INS gyro x 索引
#define INS_GYRO_Y_ADDRESS_OFFSET          1      // INS gyro y 索引
#define INS_GYRO_Z_ADDRESS_OFFSET          2      // INS gyro z 索引
#define GIMBAL_YAW_MIT_INDEX               0U     // MIT 电机反馈数组中 yaw 电机索引
#define GIMBAL_PITCH_MIT_INDEX             1u     // MIT 电机反馈数组中 pitch 电机索引
#define GIMBAL_MIT_FEEDBACK_INIT_DELAY     100U   // MIT 电机反馈初始化延时，单位 ms

/* ========================= 拨弹控制参数 ========================= */
//ps:拨弹盘减速比4，如有改动前馈力矩也得跟着改
#define SHOOT_STRUM_SINGLE_STEP_RAD        (PI * 1.0f) // 单次拨弹角度，单位 rad
#define SHOOT_STRUM_LONG_PRESS_MS          500U        // 长按判定时间，单位 ms
#define SHOOT_STRUM_DIRECTION              1.0f        // 拨弹方向符号
#define SHOOT_STRUM_FDB_TIMEOUT            100U        // 拨弹反馈超时，单位 ms
#define SHOOT_STRUM_SINGLE_TORQUE_FF_NM    7.0f        // 单击拨弹前馈力矩，单位 N*m
#define SHOOT_STRUM_SINGLE_FF_TIME_MS      200U         // 单击拨弹前馈持续时间，单位 ms
#define SHOOT_STRUM_SINGLE_FF_RELEASE_MS   80U          // 单击拨弹前馈退出斜坡时间，单位 ms
#define SHOOT_STRUM_SINGLE_FF_FILTER_ALPHA 1.0f       // 单击拨弹前馈低通滤波系数

/* ========================= 云台机械限位与初始化配置 ========================= */
#define YAW_MAX_RELATIVE_ANGLE             1.5707963f  // yaw 相对角上限
#define YAW_MIN_RELATIVE_ANGLE            -1.5707963f  // yaw 相对角下限
#define PITCH_MAX_RELATIVE_ANGLE           0.1f        // pitch 软件上限，单位 rad
#define PITCH_MIN_RELATIVE_ANGLE          -0.71f       // pitch 软件下限，单位 rad
#define HALF_ECD_RANGE                     4096        // 编码器半量程
#define ECD_RANGE                          8191        // 编码器总量程
#define GIMBAL_INIT_ANGLE_ERROR            0.1f        // 初始化目标角允许误差
#define GIMBAL_INIT_STOP_TIME              100         // 初始化停止判定时间
#define GIMBAL_INIT_TIME                   6000        // 初始化总超时时间
#define GIMBAL_CALI_REDUNDANT_ANGLE        0.1f        // 校准冗余角度
#define GIMBAL_INIT_PITCH_SPEED            0.004f      // pitch 初始化速度
#define GIMBAL_INIT_YAW_SPEED              0.005f      // yaw 初始化速度
#define GIMBAL_CALI_MOTOR_SET              8000        // 校准时电机输出
#define GIMBAL_CALI_STEP_TIME              2000        // 校准步骤持续时间
#define GIMBAL_CALI_GYRO_LIMIT             0.1f        // 校准静止角速度阈值
#define GIMBAL_CALI_PITCH_MAX_STEP         1           // pitch 最大角校准步骤
#define GIMBAL_CALI_PITCH_MIN_STEP         2           // pitch 最小角校准步骤
#define GIMBAL_CALI_YAW_MAX_STEP           3           // yaw 最大角校准步骤
#define GIMBAL_CALI_YAW_MIN_STEP           4           // yaw 最小角校准步骤
#define GIMBAL_CALI_START_STEP             GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP               5
#define GIMBAL_MOTIONLESS_RC_DEADLINE      10          // 进入静止行为的遥控死区阈值
#define GIMBAL_MOTIONLESS_TIME_MAX         3000        // 静止行为最大保持时间

#define INIT_YAW_SET                       0.0f        // yaw 初始化目标角
#define INIT_PITCH_SET                     0.0f        // pitch 初始化目标角

/* ========================= pitch 重力补偿配置 ========================= */
#define PITCH_GRAVITY_COMP_MASS_KG         PITCH_EQ_MASS_KG // pitch 重力补偿质量
#define PITCH_GRAVITY_COMP_COM_FORWARD_M   0.04f            // 质心前向距离
#define PITCH_GRAVITY_COMP_COM_UP_M        0.02f            // 质心上向距离
#define PITCH_GRAVITY_COMP_OUTPUT_LIMIT    (T_MAX / 5.0f)   // 重力补偿输出限幅

/* ========================= 射速估计参数 ========================= */
#define SHOOT_FRIC_WHEEL_RADIUS_M          0.022f
#define SHOOT_BULLET_42MM_MASS_KG          0.0404f
#define SHOOT_FRIC_ROTATING_MASS_KG        0.1793f
#define SHOOT_FRIC_ROTATING_INERTIA_KGM2   (0.5f * SHOOT_FRIC_ROTATING_MASS_KG * SHOOT_FRIC_WHEEL_RADIUS_M * SHOOT_FRIC_WHEEL_RADIUS_M)
#define SHOOT_BULLET_SPEED_EST_TRIGGER_DROP_RPM 600.0f
#define SHOOT_BULLET_SPEED_EST_MIN_SPEED_RATIO  0.85f
#define SHOOT_BULLET_SPEED_EST_WINDOW_MS   20U
#define SHOOT_BULLET_SPEED_EST_COEFF_MPS_PER_RPM \
    ((3.0f * SHOOT_FRIC_ROTATING_INERTIA_KGM2 * 2.0f * PI) / \
     (60.0f * SHOOT_BULLET_42MM_MASS_KG * SHOOT_FRIC_WHEEL_RADIUS_M))
#define SHOOT_FIRE_DETECT_TRIGGER_DROP_RPM 600.0f // 开火检测掉速阈值，单位 rpm
#define SHOOT_FIRE_DETECT_MIN_SPEED_RATIO  0.85f  // 摩擦轮达到目标转速比例后允许检测
#define SHOOT_FIRE_DETECT_CURRENT_A        2.0f   // 开火检测反馈电流阈值，单位 A
#define SHOOT_FIRE_DETECT_WINDOW_MS        12U    // 掉速后等待电流响应的窗口，单位 ms
#define SHOOT_FIRE_DETECT_LATCH_MS         50U    // 开火检测结果保持时间，单位 ms

/* ========================= 串口与裁判系统配置 ========================= */
#define USART_RX_BUF_LENGHT                64    // 串口接收缓冲区长度
#define REFEREE_FIFO_BUF_LENGTH            1024  // 裁判系统 FIFO 长度
#ifndef REF_PROTOCOL_FRAME_MAX_SIZE
#define REF_PROTOCOL_FRAME_MAX_SIZE        192   // 裁判系统最大帧长
#endif

/* ========================= CAN 电机 ID 配置 ========================= */
/* CAN1: yaw DM MIT motor and strum DM MIT motor */
#define DM_YAW_CAN_ID                       0X01
#define DM_YAW_MASTER_ID                    0X51

/* CAN2: pitch DM MIT motor */
#define DM_PIT_CAN_ID                       0X02
#define DM_PIT_MASTER_ID                    0X52
#define CAN_FRIC1_ID                        0X201 // 摩擦轮 1 电机 ID
#define CAN_FRIC2_ID                        0X202 // 摩擦轮 2 电机 ID
#define CAN_FRIC3_ID                        0X203 // 摩擦轮 3 电机 ID
#define CAN_STRUM_ID                        0X204 // 拨弹 DJI 电机反馈 ID

#define CAN_CHASSIS_ALL_ID                0x1FF  // 底盘 3508 电机电流控制广播 ID
#define CAN_M1_3508_ID                    0x205  // 底盘 3508 电机 1 反馈 ID
#define CAN_M2_3508_ID                    0x206  // 底盘 3508 电机 2 反馈 ID
#define CAN_M3_3508_ID                    0x207  // 底盘 3508 电机 3 反馈 ID
#define CAN_M4_3508_ID                    0x208  // 底盘 3508 电机 4 反馈 ID

#define CHASSIS_MODULE_NUM                4U                 // 底盘电机数量

/* ========================= 底盘几何与电机映射 ========================= */
/* 十字全向轮底盘：x 轴向右，y 轴向前，轮距 0.46 m */
#define CHASSIS_HALF_WHEELBASE            0.23f              // 底盘半轮距，用于旋转速度分量计算，单位 m
#define CHASSIS_OMNI_ROTATE_RADIUS        CHASSIS_HALF_WHEELBASE // 底盘旋转半径，单位 m
#define Wheel_Radius                      0.075f             // 底盘轮半径，单位 m

#define CHASSIS_WHEEL_205_DIRECTION       1.0f               // 205 号底盘电机方向系数
#define CHASSIS_WHEEL_206_DIRECTION       -1.0f              // 206 号底盘电机方向系数
#define CHASSIS_WHEEL_207_DIRECTION       -1.0f              // 207 号底盘电机方向系数
#define CHASSIS_WHEEL_208_DIRECTION       1.0f               // 208 号底盘电机方向系数

/* 底盘电机 CAN ID 与数组索引映射：205 后轮，206 右轮，207 前轮，208 左轮 */
#define WHEEL_REAR_205                    0                  // 205 号后轮数组索引
#define WHEEL_RIGHT_206                   1                  // 206 号右轮数组索引
#define WHEEL_FRONT_207                   2                  // 207 号前轮数组索引
#define WHEEL_LEFT_208                    3                  // 208 号左轮数组索引

/* ========================= 底盘任务周期与输入映射 ========================= */
#define CHASSIS_TASK_INIT_TIME            500                // 底盘任务启动延时，单位 ms
#define CHASSIS_CONTROL_TIME_MS           1                  // 底盘控制周期，单位 ms
#define CHASSIS_CONTROL_TIME              0.001f             // 底盘控制周期，单位 s
#define CHASSIS_CONTROL_FREQUENCE         1000.0f            // 底盘控制频率，单位 Hz

#define CHASSIS_X_CHANNEL                 1                  // 底盘 x 方向遥控通道
#define CHASSIS_Y_CHANNEL                 0                  // 底盘 y 方向遥控通道
#define CHASSIS_WZ_CHANNEL                2                  // 底盘旋转遥控通道
#define CHASSIS_MODE_CHANNEL              0                  // 底盘模式切换遥控通道

#define CHASSIS_VX_RC_SEN                 0.005625f          // 底盘 x 方向遥控灵敏度，单位 (m/s)/遥控计数
#define CHASSIS_VY_RC_SEN                 0.005625f          // 底盘 y 方向遥控灵敏度，单位 (m/s)/遥控计数
#define CHASSIS_WZ_RC_SEN                 0.01f              // 底盘旋转摇杆灵敏度，单位 rad/s/遥控计数
#define CHASSIS_YAW_HOLD_RC_SEN           0.004f             // yaw 保持模式摇杆积分灵敏度，单位 rad/s/遥控计数
#define CHASSIS_RC_DEADLINE               25                 // 底盘遥控死区

#define CHASSIS_FRONT_KEY                 KEY_PRESSED_OFFSET_W // 底盘前进键
#define CHASSIS_BACK_KEY                  KEY_PRESSED_OFFSET_S // 底盘后退键
#define CHASSIS_LEFT_KEY                  KEY_PRESSED_OFFSET_A // 底盘左移键
#define CHASSIS_RIGHT_KEY                 KEY_PRESSED_OFFSET_D // 底盘右移键

/* ========================= 底盘速度上限与规划限幅 ========================= */
#define NORMAL_MAX_CHASSIS_SPEED_X        5.2f               // 底盘 x 方向最大速度，单位 m/s
#define NORMAL_MAX_CHASSIS_SPEED_Y        5.2f               // 底盘 y 方向最大速度，单位 m/s
#define MAX_WHEEL_SPEED                   5.2f               // 单轮目标速度上限，单位 m/s
#define CHASSIS_SPIN_SPEED                25.0f              // 小陀螺固定旋转速度，单位 rad/s

#define CHASSIS_ACCEL_X_NUM               0.1f               // x 方向遥控速度一阶滤波系数，无量纲
#define CHASSIS_ACCEL_Y_NUM               0.1f               // y 方向遥控速度一阶滤波系数，无量纲
#define CHASSIS_MAX_ACCEL                 10.0f              // 底盘平移最大加速度，单位 m/s^2
#define CHASSIS_MAX_JERK                  35.0f              // 底盘平移最大加加速度，单位 m/s^3
#define CHASSIS_STOP_DECEL                28.0f              // 底盘松杆停车减速度，单位 m/s^2
#define CHASSIS_STOP_JERK                 1000.0f            // 底盘松杆停车减速度变化率，单位 m/s^3
#define CHASSIS_WZ_MAX_SPEED              12.0f              // 底盘旋转最大角速度，单位 rad/s
#define CHASSIS_WZ_MAX_ACCEL              18.0f              // 底盘旋转最大角加速度，单位 rad/s^2
#define CHASSIS_WZ_MAX_JERK               160.0f             // 底盘旋转最大角加加速度，单位 rad/s^3
#define CHASSIS_LAT_ACCEL_LIMIT           9.0f               // 底盘高速转弯横向加速度上限，单位 m/s^2
#define CHASSIS_LAT_SPEED_EPS             1.0f               // 横向加速度限幅启用的最小平移速度，单位 m/s

/* ========================= 底盘量纲换算 ========================= */
#define CHASSIS_CURRENT_CMD_FULL_SCALE    16384.0f           // 3508 电流命令满量程，单位 电流命令计数
#define CHASSIS_CURRENT_FULL_SCALE_A      20.0f              // 3508 电流满量程，单位 A
#define CHASSIS_CURRENT_CMD_TO_A          (CHASSIS_CURRENT_FULL_SCALE_A / CHASSIS_CURRENT_CMD_FULL_SCALE) // 电流命令转 A 系数，单位 A/电流命令计数

#define MPS_to_RPM                        2445.72f           // 轮速 m/s 转电机 rpm 系数，单位 rpm/(m/s)
#define CHASSIS_RPM_TO_RAD_PER_SEC        0.104719755f       // 电机 rpm 转 rad/s 系数，单位 (rad/s)/rpm

/* ========================= 底盘轮速 PID 与积分清理 ========================= */
#define CHASSIS_SPEED_PI_KP               1600.0f            // 底盘轮速 PID 比例系数，单位 电流命令计数/(m/s)
#define CHASSIS_SPEED_PI_KI               45.0f              // 底盘轮速 PID 积分系数，单位 电流命令计数/m
#define CHASSIS_SPEED_PI_KD               350.0f             // 底盘轮速 PID 微分系数，单位 电流命令计数/(m/s^2)
#define CHASSIS_SPEED_PI_MAX_OUT          4800.0f            // 底盘轮速 PID 输出限幅，单位 电流命令计数
#define CHASSIS_SPEED_PI_MAX_IOUT         650.0f             // 底盘轮速 PID 积分限幅，单位 电流命令计数
#define CHASSIS_ZERO_SPEED_I_CLEAR_CYCLES 10U                // 零输入零速保持后清理轮速 PI 积分的周期数，1ms 周期下为 10ms

/* ========================= 底盘电流与功率限幅 ========================= */
#define CHASSIS_CURRENT_BASE_LIMIT_A      4.5f               // 底盘单电机基础电流限幅，单位 A
#define CHASSIS_CURRENT_DYNAMIC_POOL_A    8.0f               // 底盘四电机共享动态电流池，单位 A
#define CHASSIS_CURRENT_SINGLE_MAX_A      9.0f               // 底盘单电机动态限幅上限，单位 A
#define CHASSIS_CURRENT_LIMIT_SLEW_A_PER_S 200.0f            // 底盘单电机限幅变化率，单位 A/s
#define CHASSIS_CURRENT_DEMAND_EPS_A      0.05f              // 底盘动态限幅需求电流阈值，单位 A
#define M3505_MOTOR_SPEED_PID_MAX_OUT     16000.0f           // 底盘 3508 电流输出绝对限幅，单位 电流命令计数

/* ========================= 底盘 yaw 控制 ========================= */
#define CHASSIS_RETURN_TARGET             ( 0.72f)           // 底盘回正目标角，单位 rad
#define CHASSIS_RETURN_OFFSET             ( 1.15f)           // 底盘回正坐标旋转偏置，单位 rad
#define YAW_RETURN_PID_KP                 800.0f             // 底盘回正角度 PID 比例系数
#define YAW_RETURN_PID_KI                 0.08f              // 底盘回正角度 PID 积分系数
#define YAW_RETURN_PID_KD                 20.0f              // 底盘回正角度 PID 微分系数
#define YAW_RETURN_PID_MAX_OUT            600.0f             // 底盘回正角度 PID 输出限幅，单位 PID输出计数
#define YAW_RETURN_PID_MAX_IOUT           60.0f              // 底盘回正角度 PID 积分限幅，单位 PID输出计数
#define CHASSIS_RETURN_WZ_SCALE           0.006f             // 底盘回正 PID 输出转角速度系数，单位 (rad/s)/PID输出计数
#define CHASSIS_ANGLE_PD_KP               7.0f               // 底盘角度 PD 比例系数，单位 (rad/s)/rad
#define CHASSIS_ANGLE_PD_KD               0.4f               // 底盘角度 PD 微分系数
#define CHASSIS_ANGLE_PD_DEADBAND         0.012f             // 底盘角度 PD 误差死区，单位 rad
#define CHASSIS_ANGLE_PD_MAX_OUT          8.0f               // 底盘角度 PD 输出限幅，单位 rad/s
#define CHASSIS_YAW_RATE_FEEDBACK_SIGN    -1.0f              // 底盘 yaw 陀螺仪角速度反馈方向系数

/* ========================= 底盘整车前馈与速度制动补偿 ========================= */
#define ROBOT_MASS                        8.0f               // 整车质量，单位 kg
#define CHASSIS_BODY_FF_YAW_INERTIA_KGM2  0.39f              // 底盘 yaw 轴转动惯量前馈参数，单位 kg*m^2
#define CHASSIS_BODY_FF_YAW_ACCEL_LIMIT   60.0f              // 底盘 yaw 规划角加速度限幅，单位 rad/s^2
#define CHASSIS_BODY_FF_MAX_CURRENT_CMD   3200.0f            // 底盘整车动力学前馈单轮电流命令限幅
#define CHASSIS_BODY_VEL_BRAKE_ENABLE     1U                 // 1: 使能整车速度误差制动补偿
#define CHASSIS_BODY_VEL_BRAKE_LOW_SPEED  0.1f               // 平移制动补偿低速增益结束速度，单位 m/s
#define CHASSIS_BODY_VEL_BRAKE_HIGH_SPEED 0.15f               // 平移制动补偿高速增益开始速度，单位 m/s
#define CHASSIS_BODY_VEL_BRAKE_XY_KP_LOW  150.0f              // 低速平移速度误差转期望减速度系数，单位 (m/s^2)/(m/s)
#define CHASSIS_BODY_VEL_BRAKE_XY_KP_HIGH 300.2f             // 高速平移速度误差转期望减速度系数，单位 (m/s^2)/(m/s)
#define CHASSIS_BODY_VEL_BRAKE_WZ_KP      6.0f               // 旋转速度误差转期望角减速度系数，单位 (rad/s^2)/(rad/s)
#define CHASSIS_BODY_VEL_BRAKE_SPEED_EPS  0.05f              // 平移制动补偿启用速度死区，单位 m/s
#define CHASSIS_BODY_VEL_BRAKE_WZ_EPS     0.05f              // 旋转制动补偿启用角速度死区，单位 rad/s
#define CHASSIS_BODY_VEL_BRAKE_ACCEL_LIMIT 6.0f              // 平移制动补偿期望减速度限幅，单位 m/s^2
#define CHASSIS_BODY_VEL_BRAKE_WZ_ACCEL_LIMIT 30.0f          // 旋转制动补偿期望角减速度限幅，单位 rad/s^2
#define CHASSIS_BODY_VEL_BRAKE_CURRENT_CMD_LIMIT 2400.0f     // 整车速度误差制动补偿单轮电流命令限幅

/* ========================= 底盘单轮前馈与制动状态 ========================= */
#define M3508_TORQUE_CONSTANT             0.3f               // M3508 电机转矩常数，用于转矩换算电流，单位 N*m/A
#define M3508_REDUCTION_RATIO             19.2032f           // M3508 减速比
#define CHASSIS_EFFICIENCY                0.92f              // 底盘传动效率
#define M3508_MAX_CONT_TORQUE             2.8f               // M3508 电机持续输出转矩上限，单位 N*m
#define CHASSIS_FF_VISCOUS_GAIN           0.100f             // 黏性摩擦前馈系数，单位 电流命令计数/(m/s)
#define CHASSIS_FF_COULOMB_CURRENT        0.80f              // 库仑摩擦前馈电流，单位 电流命令计数
#define CHASSIS_FF_COULOMB_SPEED_EPS      0.08f              // 库仑摩擦平滑速度阈值，单位 m/s
#define CHASSIS_FF_STATIC_CURRENT         40.0f              // 静摩擦前馈电流，单位 电流命令计数
#define CHASSIS_FF_STATIC_SPEED_EPS       0.05f              // 静摩擦平滑速度阈值，单位 m/s
#define CHASSIS_BRAKE_FRICTION_FF_SCALE   0.0f               // 制动状态摩擦前馈保留比例
#define CHASSIS_BRAKE_FF_CURRENT_A        0.0f               // 制动状态固定制动前馈电流，单位 A
#define CHASSIS_BRAKE_FF_SPEED_EPS        0.005f             // 制动前馈方向平滑速度阈值，单位 m/s
#define CHASSIS_BRAKE_ENTER_SPEED_EPS     0.02f              // 制动进入实测轮速阈值，单位 m/s
#define CHASSIS_BRAKE_RELEASE_SPEED_EPS   0.002f             // 制动释放实测轮速阈值，单位 m/s

/* ========================= MIT 协议参数范围 ========================= */
#define P_MIN                              -12.5663704f // 位置最小值
#define P_MAX                               12.5663704f // 位置最大值
#define V_MIN                              -30          // 速度最小值
#define V_MAX                               30          // 速度最大值
#define KP_MIN                              0.0         // Kp 最小值
#define KP_MAX                              500.0       // Kp 最大值
#define KD_MIN                              0.0         // Kd 最小值
#define KD_MAX                              5.0         // Kd 最大值
#define T_MIN                              -10.0f       // 力矩最小值
#define T_MAX                               10.0f       // 力矩最大值

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PARAM_H */
