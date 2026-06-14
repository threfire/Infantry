/**
  * @file       target_curve.h
  * @brief      云台目标曲线生成器接口
  * @note       提供常值、三角函数、周期波形、阶跃、斜坡和手动 S 型速度曲线。
  */
#ifndef TARGET_CURVE_H
#define TARGET_CURVE_H

#include <stdint.h>
#include "robot_param.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TARGET_PI
#define TARGET_PI PI
#endif

typedef enum
{
    TARGET_CURVE_CONST = 0,
    TARGET_CURVE_SINE,
    TARGET_CURVE_COSINE,
    TARGET_CURVE_SQUARE,
    TARGET_CURVE_TRIANGLE,
    TARGET_CURVE_SAW_UP,
    TARGET_CURVE_SAW_DOWN,
    TARGET_CURVE_TRAPEZOID,
    TARGET_CURVE_STEP,
    TARGET_CURVE_RAMP,

    /* 单次点到点 S 型轨迹。 */
    TARGET_CURVE_S_CURVE_STEP,

    /* 手动输入整形模式，输入 [-1, 1]，输出平滑 position / velocity / acceleration。 */
    TARGET_CURVE_MANUAL_S_VEL,
} TargetCurveType_t;

typedef struct
{
    float position;
    float velocity;
    float acceleration;
} TargetCurveState_t;

typedef struct
{
    TargetCurveType_t type;

    /* 通用谐波和周期参数。 */
    float amplitude;
    float frequency_hz;
    float phase_rad;
    float offset;

    /* 方波参数。 */
    float duty;

    /* 梯形波参数。 */
    float rise_ratio;
    float high_ratio;
    float fall_ratio;

    /* step / ramp / s-curve-step 参数。 */
    float step_time_s;
    float start_value;
    float end_value;
    float slope;
    float move_time_s;

    /* 手动 S 型速度整形参数。 */
    float manual_input;
    float manual_max_vel;
    float manual_max_acc;
    float manual_max_jerk;
    float manual_vel_follow_k;

    /* 通用运行时间。 */
    float time_s;

    /* 输出限幅。 */
    uint8_t limit_enable;
    float min_output;
    float max_output;

    /* 当前状态。 */
    TargetCurveState_t state;
} TargetCurve_t;

/**
  * @brief          初始化目标曲线对象
  * @param[in,out]  obj: 目标曲线对象指针
  * @retval         none
  */
void TargetCurve_Init(TargetCurve_t *obj);

/**
  * @brief          重置目标曲线时间和状态
  * @param[in,out]  obj: 目标曲线对象指针
  * @retval         none
  */
void TargetCurve_Reset(TargetCurve_t *obj);

/**
  * @brief          设置目标曲线输出限幅
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      enable: 限幅使能
  * @param[in]      min_output: 输出下限
  * @param[in]      max_output: 输出上限
  * @retval         none
  */
void TargetCurve_SetOutputLimit(TargetCurve_t *obj,
                                uint8_t enable,
                                float min_output,
                                float max_output);

/**
  * @brief          设置目标曲线类型
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      type: 曲线类型
  * @retval         none
  */
void TargetCurve_SetType(TargetCurve_t *obj, TargetCurveType_t type);

/**
  * @brief          设置目标曲线幅值
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      amplitude: 幅值
  * @retval         none
  */
void TargetCurve_SetAmplitude(TargetCurve_t *obj, float amplitude);

/**
  * @brief          设置目标曲线频率
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      frequency_hz: 频率，单位 Hz
  * @retval         none
  */
void TargetCurve_SetFrequency(TargetCurve_t *obj, float frequency_hz);

/**
  * @brief          设置目标曲线相位
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      phase_rad: 相位，单位 rad
  * @retval         none
  */
void TargetCurve_SetPhase(TargetCurve_t *obj, float phase_rad);

/**
  * @brief          设置目标曲线偏置
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      offset: 偏置值
  * @retval         none
  */
void TargetCurve_SetOffset(TargetCurve_t *obj, float offset);

/**
  * @brief          配置常值目标曲线
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      value: 常值输出
  * @retval         none
  */
void TargetCurve_SetConstant(TargetCurve_t *obj, float value);

/**
  * @brief          配置正弦目标曲线
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      amplitude: 幅值
  * @param[in]      frequency_hz: 频率，单位 Hz
  * @param[in]      phase_rad: 相位，单位 rad
  * @param[in]      offset: 偏置值
  * @retval         none
  */
void TargetCurve_SetSine(TargetCurve_t *obj,
                         float amplitude,
                         float frequency_hz,
                         float phase_rad,
                         float offset);

/**
  * @brief          配置余弦目标曲线
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      amplitude: 幅值
  * @param[in]      frequency_hz: 频率，单位 Hz
  * @param[in]      phase_rad: 相位，单位 rad
  * @param[in]      offset: 偏置值
  * @retval         none
  */
void TargetCurve_SetCosine(TargetCurve_t *obj,
                           float amplitude,
                           float frequency_hz,
                           float phase_rad,
                           float offset);

/**
  * @brief          配置方波目标曲线
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      amplitude: 幅值
  * @param[in]      frequency_hz: 频率，单位 Hz
  * @param[in]      phase_rad: 相位，单位 rad
  * @param[in]      offset: 偏置值
  * @param[in]      duty: 占空比
  * @retval         none
  */
void TargetCurve_SetSquare(TargetCurve_t *obj,
                           float amplitude,
                           float frequency_hz,
                           float phase_rad,
                           float offset,
                           float duty);

/**
  * @brief          配置三角波目标曲线
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      amplitude: 幅值
  * @param[in]      frequency_hz: 频率，单位 Hz
  * @param[in]      phase_rad: 相位，单位 rad
  * @param[in]      offset: 偏置值
  * @retval         none
  */
void TargetCurve_SetTriangle(TargetCurve_t *obj,
                             float amplitude,
                             float frequency_hz,
                             float phase_rad,
                             float offset);

/**
  * @brief          配置上升锯齿波目标曲线
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      amplitude: 幅值
  * @param[in]      frequency_hz: 频率，单位 Hz
  * @param[in]      phase_rad: 相位，单位 rad
  * @param[in]      offset: 偏置值
  * @retval         none
  */
void TargetCurve_SetSawUp(TargetCurve_t *obj,
                          float amplitude,
                          float frequency_hz,
                          float phase_rad,
                          float offset);

/**
  * @brief          配置下降锯齿波目标曲线
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      amplitude: 幅值
  * @param[in]      frequency_hz: 频率，单位 Hz
  * @param[in]      phase_rad: 相位，单位 rad
  * @param[in]      offset: 偏置值
  * @retval         none
  */
void TargetCurve_SetSawDown(TargetCurve_t *obj,
                            float amplitude,
                            float frequency_hz,
                            float phase_rad,
                            float offset);

/**
  * @brief          配置梯形波目标曲线
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      amplitude: 幅值
  * @param[in]      frequency_hz: 频率，单位 Hz
  * @param[in]      phase_rad: 相位，单位 rad
  * @param[in]      offset: 偏置值
  * @param[in]      rise_ratio: 上升段比例
  * @param[in]      high_ratio: 高电平段比例
  * @param[in]      fall_ratio: 下降段比例
  * @retval         none
  */
void TargetCurve_SetTrapezoid(TargetCurve_t *obj,
                              float amplitude,
                              float frequency_hz,
                              float phase_rad,
                              float offset,
                              float rise_ratio,
                              float high_ratio,
                              float fall_ratio);

/**
  * @brief          配置阶跃目标曲线
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      start_value: 起始值
  * @param[in]      end_value: 目标值
  * @param[in]      step_time_s: 阶跃时间，单位 s
  * @retval         none
  */
void TargetCurve_SetStep(TargetCurve_t *obj,
                         float start_value,
                         float end_value,
                         float step_time_s);

/**
  * @brief          配置斜坡目标曲线
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      start_value: 起始值
  * @param[in]      slope: 斜率
  * @param[in]      start_time_s: 起始时间，单位 s
  * @retval         none
  */
void TargetCurve_SetRamp(TargetCurve_t *obj,
                         float start_value,
                         float slope,
                         float start_time_s);

/**
  * @brief          配置单次 S 型位置目标曲线
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      start_value: 起始值
  * @param[in]      end_value: 目标值
  * @param[in]      start_time_s: 起始时间，单位 s
  * @param[in]      move_time_s: 运动时间，单位 s
  * @retval         none
  */
void TargetCurve_SetSCurveStep(TargetCurve_t *obj,
                               float start_value,
                               float end_value,
                               float start_time_s,
                               float move_time_s);

/**
  * @brief          配置手动输入 S 型速度整形曲线
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      init_position: 初始位置
  * @param[in]      max_vel: 最大速度
  * @param[in]      max_acc: 最大加速度
  * @param[in]      max_jerk: 最大 jerk
  * @param[in]      vel_follow_k: 速度跟随系数
  * @retval         none
  */
void TargetCurve_SetManualSVel(TargetCurve_t *obj,
                               float init_position,
                               float max_vel,
                               float max_acc,
                               float max_jerk,
                               float vel_follow_k);

/**
  * @brief          设置手动归一化输入
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      input_norm: 归一化输入，范围 [-1, 1]
  * @retval         none
  */
void TargetCurve_SetManualInput(TargetCurve_t *obj, float input_norm);

/**
  * @brief          计算指定时间的目标曲线状态
  * @param[in]      obj: 目标曲线对象指针
  * @param[in]      time_s: 指定时间，单位 s
  * @retval         曲线状态
  */
TargetCurveState_t TargetCurve_CalcStateAtTime(const TargetCurve_t *obj, float time_s);

/**
  * @brief          按周期更新目标曲线状态
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      dt_s: 控制周期，单位 s
  * @retval         更新后的曲线状态
  */
TargetCurveState_t TargetCurve_UpdateState(TargetCurve_t *obj, float dt_s);

/**
  * @brief          计算指定时间的目标曲线位置输出
  * @param[in]      obj: 目标曲线对象指针
  * @param[in]      time_s: 指定时间，单位 s
  * @retval         位置输出
  */
float TargetCurve_CalcAtTime(const TargetCurve_t *obj, float time_s);

/**
  * @brief          按周期更新并返回目标曲线位置输出
  * @param[in,out]  obj: 目标曲线对象指针
  * @param[in]      dt_s: 控制周期，单位 s
  * @retval         位置输出
  */
float TargetCurve_Update(TargetCurve_t *obj, float dt_s);

/**
  * @brief          获取目标曲线当前位置输出
  * @param[in]      obj: 目标曲线对象指针
  * @retval         当前位置输出
  */
float TargetCurve_GetOutput(const TargetCurve_t *obj);

/**
  * @brief          获取目标曲线当前速度输出
  * @param[in]      obj: 目标曲线对象指针
  * @retval         当前速度输出
  */
float TargetCurve_GetVelocity(const TargetCurve_t *obj);

/**
  * @brief          获取目标曲线当前加速度输出
  * @param[in]      obj: 目标曲线对象指针
  * @retval         当前加速度输出
  */
float TargetCurve_GetAcceleration(const TargetCurve_t *obj);

#ifdef __cplusplus
}
#endif

#endif
