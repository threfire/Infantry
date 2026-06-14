#ifndef ADRC_H
#define ADRC_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机简化模型参数
 *
 * 这组参数只在“根据电机参数自动估算 b0”时使用。
 * 如果你已经通过实验辨识得到了 b0，也可以直接填 b0，
 * 此时 motor 可以置为 0。
 */
typedef struct
{
    float torque_constant;   /* 力矩常数，常用单位 N*m/A */
    float inertia;           /* 转动惯量，常用单位 kg*m^2 */
} ADRC_MotorModel;

/**
 * @brief ADRC 初始化配置
 *
 * 常见用法有两种：
 * 1. 直接给出 b0
 * 2. 提供 motor，由库内部自动用 torque_constant / inertia 估算 b0
 */
typedef struct
{
    float sample_time_s;              /* 控制周期，单位秒 */
    float b0;                         /* 被控对象输入增益估计值，>0 时优先使用 */
    float controller_bandwidth;       /* 控制器带宽 wc */
    float observer_bandwidth_ratio;   /* 观测器带宽相对控制器带宽的倍数 */
    float tracking_gain;              /* 目标跟踪器增益，0 表示直接使用目标值 */
    float output_min;                 /* 输出下限 */
    float output_max;                 /* 输出上限 */
    float output_rate_limit;          /* 输出变化率上限，单位为“输出单位/秒” */
    float error_linear_zone;          /* fal 函数线性区间阈值 */
    float alpha1;                     /* 控制律 fal 指数 */
    float alpha2;                     /* ESO fal 指数 */
    const ADRC_MotorModel *motor;     /* 电机模型，可为空 */
} ADRC_Config;

/**
 * @brief ADRC 控制器实例
 *
 * 一个电机建议对应一个独立的 ADRC_Controller。
 * 如果一块 MCU 同时控制多个电机，就创建多个实例分别使用。
 */
typedef struct
{
    float sample_time_s;              /* 控制周期 */
    float b0;                         /* 输入增益估计 */
    float controller_bandwidth;       /* 控制器带宽 wc */
    float observer_bandwidth;         /* 观测器带宽 wo */
    float observer_bandwidth_ratio;   /* 观测器带宽比例 */
    float tracking_gain;              /* 跟踪器增益 */
    float output_min;                 /* 输出下限 */
    float output_max;                 /* 输出上限 */
    float output_rate_limit;          /* 输出变化率上限 */
    float error_linear_zone;          /* fal 线性区阈值 */
    float alpha1;                     /* 控制律非线性指数 */
    float alpha2;                     /* 观测器非线性指数 */

    float kp;                         /* 一阶 ADRC 控制器等效比例增益 */
    float beta1;                      /* ESO 增益 1 */
    float beta2;                      /* ESO 增益 2 */

    float z1;                         /* ESO 对输出状态的估计 */
    float z2;                         /* ESO 对总扰动的估计 */
    float tracking_state;             /* 跟踪器内部状态 */
    float last_output;                /* 上一次控制输出 */
} ADRC_Controller;

/**
 * @brief 面向工程替换的简化参数结构
 *
 * 这个结构的设计目标是尽量贴近 pid.c/h 的使用习惯：
 * 1. 初始化时一次性送入主要参数
 * 2. 后续可单独调用设参接口更新
 * 3. 每拍先更新反馈，再设置目标，再取计算结果
 *
 * 其中：
 * - max_out 对应 PID 里的最大输出，内部按 [-max_out, +max_out] 处理
 * - output_rate_limit 用于限制每秒最大输出变化量
 */
typedef struct
{
    float sample_time_s;              /* 控制周期 */
    float b0;                         /* 输入增益估计 */
    float controller_bandwidth;       /* 控制器带宽 wc */
    float observer_bandwidth_ratio;   /* 观测器带宽比例 */
    float tracking_gain;              /* 跟踪器增益 */
    float max_out;                    /* 对称输出上限 */
    float output_rate_limit;          /* 输出变化率上限 */
    float error_linear_zone;          /* fal 线性区 */
    float alpha1;                     /* 控制律 fal 指数 */
    float alpha2;                     /* ESO fal 指数 */
} adrc_param_t;

/**
 * @brief 面向工程替换的 ADRC 对象
 *
 * 这个对象公开保留 set / fdb / out / delta_out 等字段，
 * 目的是让它在调用习惯上更接近 pid_type_def：
 * - set：当前设定值
 * - fdb：当前反馈值
 * - out：本周期输出
 * - delta_out：本周期相对上一拍的变化量
 *
 * 内部仍然复用通用的 ADRC_Controller 和 ADRC_Config。
 */
typedef struct
{
    ADRC_Controller core;             /* 底层通用 ADRC 控制器 */
    ADRC_Config config;               /* 当前生效配置 */

    float set;                        /* 当前设定值 */
    float fdb;                        /* 当前反馈值 */
    float out;                        /* 当前输出 */
    float last_out;                   /* 上一拍输出 */
    float delta_out;                  /* 本拍输出变化量 */

    float disturbance;                /* ESO 估计总扰动 */
    float tracking_state;             /* 跟踪器内部状态 */
} adrc_type_def;

/**
 * @brief 返回值定义
 */
enum
{
    ADRC_OK = 0,
    ADRC_ERR_INVALID_ARG = -1,
    ADRC_ERR_INVALID_PARAM = -2
};

/**
 * @brief 默认参数
 */
#define ADRC_DEFAULT_ALPHA1              (0.5f)
#define ADRC_DEFAULT_ALPHA2              (0.25f)
#define ADRC_DEFAULT_OBSERVER_RATIO      (4.0f)
#define ADRC_DEFAULT_ERROR_LINEAR_ZONE   (1.0f)

/**
 * @brief 初始化 ADRC 控制器
 * @param controller 控制器实例指针
 * @param config 初始化配置
 * @return 成功返回 ADRC_OK，失败返回负数错误码
 */
int ADRC_Init(ADRC_Controller *controller, const ADRC_Config *config);

/**
 * @brief 根据电机参数快速生成一套基础 ADRC 配置
 *
 * 这个接口适合“先快速搭好，再进一步细调”的场景。
 * 它会根据期望响应时间、量程和输出限制生成一套初值。
 *
 * @param config 输出配置
 * @param motor 电机模型参数
 * @param sample_time_s 控制周期，单位秒
 * @param response_time_s 期望闭环响应时间，单位秒
 * @param feedback_range 被控量量程，例如最大转速 RPM
 * @param output_limit 输出绝对值上限，例如最大电流或最大占空比
 * @param output_rate_limit 输出变化率限制，0 表示不限制
 * @return 成功返回 ADRC_OK，失败返回负数错误码
 */
int ADRC_ConfigFromMotor(ADRC_Config *config,
                         const ADRC_MotorModel *motor,
                         float sample_time_s,
                         float response_time_s,
                         float feedback_range,
                         float output_limit,
                         float output_rate_limit);

/**
 * @brief 复位控制器内部状态
 * @param controller 控制器实例
 * @param measurement 当前测量值
 * @param target 当前目标值
 */
void ADRC_Reset(ADRC_Controller *controller, float measurement, float target);

/**
 * @brief 按当前工作点复位控制器内部状态
 *
 * 这个接口适合“对象已经在运行中”的场景，例如：
 * 1. 摩擦轮一直处于开启状态，中途切换控制模式
 * 2. 控制器重新初始化，但电机并没有停下来
 * 3. 希望 ESO 从当前稳态输出和稳态扰动附近开始工作
 *
 * 如果能提供当前维持稳速所需的输出 initial_output，
 * 那么对一阶 ADRC 而言，常见可取：
 * initial_disturbance ≈ -b0 * initial_output
 *
 * @param controller 控制器实例
 * @param measurement 当前测量值
 * @param target 当前目标值
 * @param initial_output 当前工作点输出，例如当前电流命令
 * @param initial_disturbance 当前工作点总扰动估计
 */
void ADRC_ResetWithState(ADRC_Controller *controller,
                         float measurement,
                         float target,
                         float initial_output,
                         float initial_disturbance);

/**
 * @brief 进行一次 ADRC 更新
 * @param controller 控制器实例
 * @param target 当前目标值
 * @param measurement 当前测量值
 * @return 本周期控制输出
 */
float ADRC_Update(ADRC_Controller *controller, float target, float measurement);

/**
 * @brief 在线修改带宽
 * @param controller 控制器实例
 * @param controller_bandwidth 新控制器带宽
 * @param observer_bandwidth_ratio 新观测器带宽比例，<=0 表示保持原比例
 */
void ADRC_SetBandwidth(ADRC_Controller *controller,
                       float controller_bandwidth,
                       float observer_bandwidth_ratio);

/**
 * @brief 设置输出上下限
 * @param controller 控制器实例
 * @param output_min 输出下限
 * @param output_max 输出上限
 */
void ADRC_SetOutputLimit(ADRC_Controller *controller, float output_min, float output_max);

/**
 * @brief 设置输出变化率限制
 * @param controller 控制器实例
 * @param output_rate_limit 输出变化率上限
 */
void ADRC_SetOutputRateLimit(ADRC_Controller *controller, float output_rate_limit);

/**
 * @brief 获取 ESO 估计的总扰动
 * @param controller 控制器实例
 * @return 扰动估计值
 */
float ADRC_GetDisturbance(const ADRC_Controller *controller);

/**
 * @brief 获取跟踪器内部状态
 * @param controller 控制器实例
 * @return 跟踪器当前内部状态
 */
float ADRC_GetTrackingState(const ADRC_Controller *controller);

/**
 * @brief 获取控制器上一次输出
 * @param controller 控制器实例
 * @return 上一次输出值
 */
float ADRC_GetLastOutput(const ADRC_Controller *controller);

/**
 * @brief 初始化 PID 风格的 ADRC 对象
 *
 * 与 PID_init 的对应关系：
 * - PID_init(...)         -> ADRC_init(...)
 *
 * 最小替换用法：
 * 1. 先准备 adrc_param_t
 * 2. 再调用 ADRC_init
 * 3. 周期内调用 ADRC_Calc
 * 4. 停机或复位时调用 ADRC_clear
 *
 * @param adrc ADRC 对象
 * @param param 初始化参数
 * @return 成功返回 ADRC_OK
 */
int ADRC_init(adrc_type_def *adrc, const adrc_param_t *param);

/**
 * @brief 在线更新参数
 *
 * 这个接口适合：
 * 1. 上位机在线调参
 * 2. 不同工况切换不同参数
 * 3. 工程里希望保留“统一设参入口”
 *
 * @param adrc ADRC 对象
 * @param param 新参数
 * @return 成功返回 ADRC_OK
 */
int ADRC_set_param(adrc_type_def *adrc, const adrc_param_t *param);

/**
 * @brief 冷启动清零
 *
 * 与 PID_clear 的对应关系：
 * - PID_clear(...)        -> ADRC_clear(...)
 *
 * 适用场景：
 * 1. 电机已经停转
 * 2. 上电初始化后希望彻底清空内部状态
 * 3. 故障恢复后重新进入控制
 *
 * @param adrc ADRC 对象
 */
void ADRC_clear(adrc_type_def *adrc);

/**
 * @brief 按当前工作点冷启动复位
 *
 * 这个接口比 ADRC_clear 更适合“当前已经有反馈值和目标值”的场景，
 * 例如刚进入控制模式时，希望 z1 先对齐到当前测量值。
 *
 * @param adrc ADRC 对象
 * @param measurement 当前反馈
 * @param target 当前目标
 */
void ADRC_reset(adrc_type_def *adrc, float measurement, float target);

/**
 * @brief 按当前工作点热启动复位
 *
 * 适合摩擦轮常开、控制模式切换这类场景。
 *
 * @param adrc ADRC 对象
 * @param measurement 当前反馈
 * @param target 当前目标
 * @param initial_output 当前维持工作点所需输出
 */
void ADRC_hot_reset(adrc_type_def *adrc,
                    float measurement,
                    float target,
                    float initial_output);

/**
 * @brief 更新反馈值
 *
 * 这个接口适合和现有工程的数据流对齐：
 * 1. 先在反馈更新函数里写入 fdb
 * 2. 再在控制函数里调用 ADRC_calc
 *
 * @param adrc ADRC 对象
 * @param fdb 当前反馈
 */
void ADRC_update_fdb(adrc_type_def *adrc, float fdb);

/**
 * @brief 更新设定值
 *
 * 如果你的工程里“目标值生成”和“控制计算”不在同一个函数里，
 * 可以先调用这个接口保存 set，再单独调用 ADRC_calc。
 *
 * @param adrc ADRC 对象
 * @param set 当前设定
 */
void ADRC_update_ref(adrc_type_def *adrc, float set);

/**
 * @brief 按已保存的设定值和反馈值执行一次计算
 *
 * 对应流程：
 * 1. ADRC_update_fdb(adrc, ref);
 * 2. ADRC_update_ref(adrc, set);
 * 3. ADRC_calc(adrc);
 *
 * @param adrc ADRC 对象
 * @return 本周期输出
 */
float ADRC_calc(adrc_type_def *adrc);

/**
 * @brief 按 PID_Calc 类似的调用方式执行一次计算
 *
 * 与 PID_Calc 的参数顺序保持一致：
 * - ref：反馈值
 * - set：设定值
 *
 * 对应关系：
 * - PID_Calc(pid, ref, set)  -> ADRC_Calc(adrc, ref, set)
 *
 * 注意：
 * - 这里 ref 表示反馈值，不是目标值
 * - 这里 set 表示设定值，含义与 pid.c/h 保持一致
 *
 * @param adrc ADRC 对象
 * @param ref 当前反馈
 * @param set 当前设定
 * @return 本周期输出
 */
float ADRC_Calc(adrc_type_def *adrc, float ref, float set);

#ifdef __cplusplus
}
#endif

#endif
