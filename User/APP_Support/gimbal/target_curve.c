/**
  * @file       target_curve.c
  * @brief      云台目标曲线生成器
  * @note       实现常值、三角函数、周期波形、阶跃、斜坡和手动 S 型速度曲线。
  */
#include "target_curve.h"
#include <math.h>

/**
  * @brief          对目标曲线数值执行区间限幅
  * @retval         none
  */
static float TargetCurve_Clamp(float x, float min_val, float max_val)
{
    if (x < min_val)
    {
        return min_val;
    }
    if (x > max_val)
    {
        return max_val;
    }
    return x;
}

/**
  * @brief          计算浮点数的小数部分
  * @retval         none
  */
static float TargetCurve_Frac(float x)
{
    return x - floorf(x);
}

/**
  * @brief          将负值钳制为 0
  * @retval         none
  */
static float TargetCurve_NonNegative(float x)
{
    return (x < 0.0f) ? 0.0f : x;
}

/**
  * @brief          将方波占空比限制到有效范围
  * @retval         none
  */
static float TargetCurve_SafeDuty(float duty)
{
    return TargetCurve_Clamp(duty, 0.01f, 0.99f);
}

/**
  * @brief          获取 2*pi 常量
  * @retval         none
  */
static float TargetCurve_TwoPi(void)
{
    return 2.0f * TARGET_PI;
}

/**
  * @brief          将弧度相位转换为周期相位
  * @retval         none
  */
static float TargetCurve_PhaseToCycle(float phase_rad)
{
    return phase_rad / TargetCurve_TwoPi();
}

/**
  * @brief          按最大步长逼近目标值
  * @retval         none
  */
static float TargetCurve_MoveToward(float current, float target, float max_delta)
{
    if (max_delta <= 0.0f)
    {
        return current;
    }

    if (current < target)
    {
        current += max_delta;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= max_delta;
        if (current < target)
        {
            current = target;
        }
    }
    return current;
}

/**
  * @brief          对目标曲线状态执行输出限幅
  * @retval         none
  */
static void TargetCurve_ApplyLimit(const TargetCurve_t *obj, TargetCurveState_t *state)
{
    if ((obj == 0) || (state == 0))
    {
        return;
    }

    if (obj->limit_enable == 0U)
    {
        return;
    }

    if (state->position > obj->max_output)
    {
        state->position = obj->max_output;

        if (state->velocity > 0.0f)
        {
            state->velocity = 0.0f;
        }
        if (state->acceleration > 0.0f)
        {
            state->acceleration = 0.0f;
        }
    }
    else if (state->position < obj->min_output)
    {
        state->position = obj->min_output;

        if (state->velocity < 0.0f)
        {
            state->velocity = 0.0f;
        }
        if (state->acceleration < 0.0f)
        {
            state->acceleration = 0.0f;
        }
    }
}

/**
  * @brief          更新手动输入 S 型速度曲线状态
  * @retval         none
  */
static TargetCurveState_t TargetCurve_UpdateManualSVel(TargetCurve_t *obj, float dt_s)
{
    TargetCurveState_t s;
    float target_vel;
    float desired_acc;
    float jerk_step;
    float v_old;

    s = obj->state;

    target_vel = TargetCurve_Clamp(obj->manual_input, -1.0f, 1.0f) * obj->manual_max_vel;

    /* 更贴近真实云台：
       人手输入先变成目标角速度，再通过加速度/jerk限制平滑跟随 */
    desired_acc = obj->manual_vel_follow_k * (target_vel - s.velocity);
    desired_acc = TargetCurve_Clamp(desired_acc, -obj->manual_max_acc, obj->manual_max_acc);

    jerk_step = obj->manual_max_jerk * dt_s;
    s.acceleration = TargetCurve_MoveToward(s.acceleration, desired_acc, jerk_step);

    v_old = s.velocity;
    s.velocity += s.acceleration * dt_s;

    /* 小误差区直接吸附，避免零附近抖动 */
    if (fabsf(target_vel - s.velocity) < (obj->manual_max_acc * dt_s * 1.2f))
    {
        if (fabsf(obj->manual_input) < 1e-4f)
        {
            s.velocity = 0.0f;
            if (fabsf(s.acceleration) < (obj->manual_max_jerk * dt_s * 1.5f))
            {
                s.acceleration = 0.0f;
            }
        }
    }

    /* 梯形积分，提高位置积分平滑性 */
    s.position += 0.5f * (v_old + s.velocity) * dt_s;

    TargetCurve_ApplyLimit(obj, &s);
    return s;
}

/**
  * @brief          初始化目标曲线对象
  * @retval         none
  */
void TargetCurve_Init(TargetCurve_t *obj)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = TARGET_CURVE_CONST;

    obj->amplitude = 0.0f;
    obj->frequency_hz = 0.0f;
    obj->phase_rad = 0.0f;
    obj->offset = 0.0f;

    obj->duty = 0.5f;

    obj->rise_ratio = 0.2f;
    obj->high_ratio = 0.3f;
    obj->fall_ratio = 0.2f;

    obj->step_time_s = 0.0f;
    obj->start_value = 0.0f;
    obj->end_value = 0.0f;
    obj->slope = 0.0f;
    obj->move_time_s = 1.0f;

    obj->manual_input = 0.0f;
    obj->manual_max_vel = 1.0f;
    obj->manual_max_acc = 5.0f;
    obj->manual_max_jerk = 50.0f;
    obj->manual_vel_follow_k = 12.0f;

    obj->time_s = 0.0f;
    obj->limit_enable = 0U;
    obj->min_output = -TARGET_PI;
    obj->max_output = TARGET_PI;

    obj->state.position = 0.0f;
    obj->state.velocity = 0.0f;
    obj->state.acceleration = 0.0f;
}

/**
  * @brief          重置目标曲线时间和状态
  * @retval         none
  */
void TargetCurve_Reset(TargetCurve_t *obj)
{
    if (obj == 0)
    {
        return;
    }

    obj->time_s = 0.0f;

    if (obj->type == TARGET_CURVE_MANUAL_S_VEL)
    {
        obj->state.position = obj->start_value;
        obj->state.velocity = 0.0f;
        obj->state.acceleration = 0.0f;
    }
    else
    {
        obj->state = TargetCurve_CalcStateAtTime(obj, 0.0f);
    }
}

/**
  * @brief          设置目标曲线输出限幅
  * @retval         none
  */
void TargetCurve_SetOutputLimit(TargetCurve_t *obj,
                                uint8_t enable,
                                float min_output,
                                float max_output)
{
    if (obj == 0)
    {
        return;
    }

    if (min_output > max_output)
    {
        float temp = min_output;
        min_output = max_output;
        max_output = temp;
    }

    obj->limit_enable = enable;
    obj->min_output = min_output;
    obj->max_output = max_output;

    TargetCurve_ApplyLimit(obj, &obj->state);
}

/**
  * @brief          设置目标曲线类型
  * @retval         none
  */
void TargetCurve_SetType(TargetCurve_t *obj, TargetCurveType_t type)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = type;
    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          设置目标曲线幅值
  * @retval         none
  */
void TargetCurve_SetAmplitude(TargetCurve_t *obj, float amplitude)
{
    if (obj == 0)
    {
        return;
    }

    obj->amplitude = TargetCurve_NonNegative(amplitude);
    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          设置目标曲线频率
  * @retval         none
  */
void TargetCurve_SetFrequency(TargetCurve_t *obj, float frequency_hz)
{
    if (obj == 0)
    {
        return;
    }

    obj->frequency_hz = TargetCurve_NonNegative(frequency_hz);
    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          设置目标曲线相位
  * @retval         none
  */
void TargetCurve_SetPhase(TargetCurve_t *obj, float phase_rad)
{
    if (obj == 0)
    {
        return;
    }

    obj->phase_rad = phase_rad;
    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          设置目标曲线偏置
  * @retval         none
  */
void TargetCurve_SetOffset(TargetCurve_t *obj, float offset)
{
    if (obj == 0)
    {
        return;
    }

    obj->offset = offset;
    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          配置常值目标曲线
  * @retval         none
  */
void TargetCurve_SetConstant(TargetCurve_t *obj, float value)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = TARGET_CURVE_CONST;
    obj->offset = value;
    obj->amplitude = 0.0f;
    obj->frequency_hz = 0.0f;
    obj->phase_rad = 0.0f;

    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          配置正弦目标曲线
  * @retval         none
  */
void TargetCurve_SetSine(TargetCurve_t *obj,
                         float amplitude,
                         float frequency_hz,
                         float phase_rad,
                         float offset)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = TARGET_CURVE_SINE;
    obj->amplitude = TargetCurve_NonNegative(amplitude);
    obj->frequency_hz = TargetCurve_NonNegative(frequency_hz);
    obj->phase_rad = phase_rad;
    obj->offset = offset;

    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          配置正弦目标曲线
  * @retval         none
  */
void TargetCurve_SetCosine(TargetCurve_t *obj,
                           float amplitude,
                           float frequency_hz,
                           float phase_rad,
                           float offset)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = TARGET_CURVE_COSINE;
    obj->amplitude = TargetCurve_NonNegative(amplitude);
    obj->frequency_hz = TargetCurve_NonNegative(frequency_hz);
    obj->phase_rad = phase_rad;
    obj->offset = offset;

    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          配置方波目标曲线
  * @retval         none
  */
void TargetCurve_SetSquare(TargetCurve_t *obj,
                           float amplitude,
                           float frequency_hz,
                           float phase_rad,
                           float offset,
                           float duty)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = TARGET_CURVE_SQUARE;
    obj->amplitude = TargetCurve_NonNegative(amplitude);
    obj->frequency_hz = TargetCurve_NonNegative(frequency_hz);
    obj->phase_rad = phase_rad;
    obj->offset = offset;
    obj->duty = TargetCurve_SafeDuty(duty);

    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          配置三角波目标曲线
  * @retval         none
  */
void TargetCurve_SetTriangle(TargetCurve_t *obj,
                             float amplitude,
                             float frequency_hz,
                             float phase_rad,
                             float offset)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = TARGET_CURVE_TRIANGLE;
    obj->amplitude = TargetCurve_NonNegative(amplitude);
    obj->frequency_hz = TargetCurve_NonNegative(frequency_hz);
    obj->phase_rad = phase_rad;
    obj->offset = offset;

    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          配置上升锯齿波目标曲线
  * @retval         none
  */
void TargetCurve_SetSawUp(TargetCurve_t *obj,
                          float amplitude,
                          float frequency_hz,
                          float phase_rad,
                          float offset)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = TARGET_CURVE_SAW_UP;
    obj->amplitude = TargetCurve_NonNegative(amplitude);
    obj->frequency_hz = TargetCurve_NonNegative(frequency_hz);
    obj->phase_rad = phase_rad;
    obj->offset = offset;

    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          配置上升锯齿波目标曲线
  * @retval         none
  */
void TargetCurve_SetSawDown(TargetCurve_t *obj,
                            float amplitude,
                            float frequency_hz,
                            float phase_rad,
                            float offset)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = TARGET_CURVE_SAW_DOWN;
    obj->amplitude = TargetCurve_NonNegative(amplitude);
    obj->frequency_hz = TargetCurve_NonNegative(frequency_hz);
    obj->phase_rad = phase_rad;
    obj->offset = offset;

    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          配置梯形波目标曲线
  * @retval         none
  */
void TargetCurve_SetTrapezoid(TargetCurve_t *obj,
                              float amplitude,
                              float frequency_hz,
                              float phase_rad,
                              float offset,
                              float rise_ratio,
                              float high_ratio,
                              float fall_ratio)
{
    float sum;

    if (obj == 0)
    {
        return;
    }

    rise_ratio = TargetCurve_NonNegative(rise_ratio);
    high_ratio = TargetCurve_NonNegative(high_ratio);
    fall_ratio = TargetCurve_NonNegative(fall_ratio);

    sum = rise_ratio + high_ratio + fall_ratio;
    if (sum > 1.0f)
    {
        rise_ratio /= sum;
        high_ratio /= sum;
        fall_ratio /= sum;
    }

    obj->type = TARGET_CURVE_TRAPEZOID;
    obj->amplitude = TargetCurve_NonNegative(amplitude);
    obj->frequency_hz = TargetCurve_NonNegative(frequency_hz);
    obj->phase_rad = phase_rad;
    obj->offset = offset;
    obj->rise_ratio = rise_ratio;
    obj->high_ratio = high_ratio;
    obj->fall_ratio = fall_ratio;

    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          配置阶跃目标曲线
  * @retval         none
  */
void TargetCurve_SetStep(TargetCurve_t *obj,
                         float start_value,
                         float end_value,
                         float step_time_s)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = TARGET_CURVE_STEP;
    obj->start_value = start_value;
    obj->end_value = end_value;
    obj->step_time_s = TargetCurve_NonNegative(step_time_s);

    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          配置斜坡目标曲线
  * @retval         none
  */
void TargetCurve_SetRamp(TargetCurve_t *obj,
                         float start_value,
                         float slope,
                         float start_time_s)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = TARGET_CURVE_RAMP;
    obj->start_value = start_value;
    obj->slope = slope;
    obj->step_time_s = TargetCurve_NonNegative(start_time_s);

    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          配置单次 S 型位置目标曲线
  * @retval         none
  */
void TargetCurve_SetSCurveStep(TargetCurve_t *obj,
                               float start_value,
                               float end_value,
                               float start_time_s,
                               float move_time_s)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = TARGET_CURVE_S_CURVE_STEP;
    obj->start_value = start_value;
    obj->end_value = end_value;
    obj->step_time_s = TargetCurve_NonNegative(start_time_s);
    obj->move_time_s = TargetCurve_NonNegative(move_time_s);

    obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
}

/**
  * @brief          配置手动输入 S 型速度整形曲线
  * @retval         none
  */
void TargetCurve_SetManualSVel(TargetCurve_t *obj,
                               float init_position,
                               float max_vel,
                               float max_acc,
                               float max_jerk,
                               float vel_follow_k)
{
    if (obj == 0)
    {
        return;
    }

    obj->type = TARGET_CURVE_MANUAL_S_VEL;

    obj->manual_input = 0.0f;
    obj->manual_max_vel = TargetCurve_NonNegative(max_vel);
    obj->manual_max_acc = TargetCurve_NonNegative(max_acc);
    obj->manual_max_jerk = TargetCurve_NonNegative(max_jerk);
    obj->manual_vel_follow_k = TargetCurve_NonNegative(vel_follow_k);

    obj->start_value = init_position;
    obj->end_value = init_position;
    obj->time_s = 0.0f;

    obj->state.position = init_position;
    obj->state.velocity = 0.0f;
    obj->state.acceleration = 0.0f;

    TargetCurve_ApplyLimit(obj, &obj->state);
}

/**
  * @brief          设置手动归一化输入
  * @retval         none
  */
void TargetCurve_SetManualInput(TargetCurve_t *obj, float input_norm)
{
    if (obj == 0)
    {
        return;
    }

    obj->manual_input = TargetCurve_Clamp(input_norm, -1.0f, 1.0f);
}

/**
  * @brief          计算指定时间的目标曲线状态
  * @retval         none
  */
TargetCurveState_t TargetCurve_CalcStateAtTime(const TargetCurve_t *obj, float time_s)
{
    TargetCurveState_t out;
    float w;
    float theta;
    float cycle;
    float phase_cycle;

    out.position = 0.0f;
    out.velocity = 0.0f;
    out.acceleration = 0.0f;

    if (obj == 0)
    {
        return out;
    }

    if (time_s < 0.0f)
    {
        time_s = 0.0f;
    }

    w = TargetCurve_TwoPi() * obj->frequency_hz;
    theta = w * time_s + obj->phase_rad;
    phase_cycle = TargetCurve_PhaseToCycle(obj->phase_rad);
    cycle = TargetCurve_Frac(obj->frequency_hz * time_s + phase_cycle);

    switch (obj->type)
    {
    case TARGET_CURVE_CONST:
        out.position = obj->offset;
        break;

    case TARGET_CURVE_SINE:
        out.position = obj->offset + obj->amplitude * sinf(theta);
        out.velocity = obj->amplitude * w * cosf(theta);
        out.acceleration = -obj->amplitude * w * w * sinf(theta);
        break;

    case TARGET_CURVE_COSINE:
        out.position = obj->offset + obj->amplitude * cosf(theta);
        out.velocity = -obj->amplitude * w * sinf(theta);
        out.acceleration = -obj->amplitude * w * w * cosf(theta);
        break;

    case TARGET_CURVE_SQUARE:
        out.position = obj->offset + ((cycle < obj->duty) ? obj->amplitude : -obj->amplitude);
        break;

    case TARGET_CURVE_TRIANGLE:
        if (cycle < 0.25f)
        {
            out.position = obj->offset + obj->amplitude * (4.0f * cycle);
            out.velocity = obj->amplitude * (4.0f * obj->frequency_hz);
        }
        else if (cycle < 0.75f)
        {
            out.position = obj->offset + obj->amplitude * (2.0f - 4.0f * cycle);
            out.velocity = obj->amplitude * (-4.0f * obj->frequency_hz);
        }
        else
        {
            out.position = obj->offset + obj->amplitude * (-4.0f + 4.0f * cycle);
            out.velocity = obj->amplitude * (4.0f * obj->frequency_hz);
        }
        break;

    case TARGET_CURVE_SAW_UP:
        out.position = obj->offset + obj->amplitude * (2.0f * cycle - 1.0f);
        out.velocity = obj->amplitude * (2.0f * obj->frequency_hz);
        break;

    case TARGET_CURVE_SAW_DOWN:
        out.position = obj->offset + obj->amplitude * (1.0f - 2.0f * cycle);
        out.velocity = obj->amplitude * (-2.0f * obj->frequency_hz);
        break;

    case TARGET_CURVE_TRAPEZOID:
    {
        float low_ratio = 1.0f - (obj->rise_ratio + obj->high_ratio + obj->fall_ratio);
        float c1 = low_ratio;
        float c2 = c1 + obj->rise_ratio;
        float c3 = c2 + obj->high_ratio;
        float c4 = c3 + obj->fall_ratio;

        if (low_ratio < 0.0f)
        {
            low_ratio = 0.0f;
            c1 = 0.0f;
            c2 = obj->rise_ratio;
            c3 = c2 + obj->high_ratio;
            c4 = c3 + obj->fall_ratio;
        }

        if (cycle < c1)
        {
            out.position = obj->offset - obj->amplitude;
        }
        else if ((cycle < c2) && (obj->rise_ratio > 1e-6f))
        {
            float u = (cycle - c1) / obj->rise_ratio;
            out.position = obj->offset + obj->amplitude * (-1.0f + 2.0f * u);
            out.velocity = obj->amplitude * (2.0f * obj->frequency_hz / obj->rise_ratio);
        }
        else if (cycle < c3)
        {
            out.position = obj->offset + obj->amplitude;
        }
        else if ((cycle < c4) && (obj->fall_ratio > 1e-6f))
        {
            float u = (cycle - c3) / obj->fall_ratio;
            out.position = obj->offset + obj->amplitude * (1.0f - 2.0f * u);
            out.velocity = obj->amplitude * (-2.0f * obj->frequency_hz / obj->fall_ratio);
        }
        else
        {
            out.position = obj->offset - obj->amplitude;
        }
        break;
    }

    case TARGET_CURVE_STEP:
        out.position = (time_s < obj->step_time_s) ? obj->start_value : obj->end_value;
        break;

    case TARGET_CURVE_RAMP:
        if (time_s < obj->step_time_s)
        {
            out.position = obj->start_value;
        }
        else
        {
            out.position = obj->start_value + obj->slope * (time_s - obj->step_time_s);
            out.velocity = obj->slope;
        }
        break;

    case TARGET_CURVE_S_CURVE_STEP:
    {
        float T = obj->move_time_s;
        float t0 = obj->step_time_s;
        float delta = obj->end_value - obj->start_value;

        if (time_s <= t0)
        {
            out.position = obj->start_value;
            out.velocity = 0.0f;
            out.acceleration = 0.0f;
        }
        else if (T <= 1e-6f)
        {
            out.position = obj->end_value;
            out.velocity = 0.0f;
            out.acceleration = 0.0f;
        }
        else if (time_s >= (t0 + T))
        {
            out.position = obj->end_value;
            out.velocity = 0.0f;
            out.acceleration = 0.0f;
        }
        else
        {
            float u = (time_s - t0) / T;
            float u2 = u * u;
            float u3 = u2 * u;
            float u4 = u3 * u;
            float u5 = u4 * u;

            /* 五次多项式 S 曲线 */
            float s   = 10.0f * u3 - 15.0f * u4 + 6.0f * u5;
            float sd  = (30.0f * u2 - 60.0f * u3 + 30.0f * u4) / T;
            float sdd = (60.0f * u - 180.0f * u2 + 120.0f * u3) / (T * T);

            out.position = obj->start_value + delta * s;
            out.velocity = delta * sd;
            out.acceleration = delta * sdd;
        }
        break;
    }

    case TARGET_CURVE_MANUAL_S_VEL:
        out = obj->state;
        break;

    default:
        out.position = 0.0f;
        break;
    }

    TargetCurve_ApplyLimit(obj, &out);
    return out;
}

/**
  * @brief          按周期更新目标曲线状态
  * @retval         none
  */
TargetCurveState_t TargetCurve_UpdateState(TargetCurve_t *obj, float dt_s)
{
    TargetCurveState_t zero = {0.0f, 0.0f, 0.0f};

    if (obj == 0)
    {
        return zero;
    }

    if (dt_s < 0.0f)
    {
        dt_s = 0.0f;
    }

    obj->time_s += dt_s;

    if (obj->type == TARGET_CURVE_MANUAL_S_VEL)
    {
        obj->state = TargetCurve_UpdateManualSVel(obj, dt_s);
    }
    else
    {
        obj->state = TargetCurve_CalcStateAtTime(obj, obj->time_s);
    }

    return obj->state;
}

/**
  * @brief          计算指定时间的目标曲线位置输出
  * @retval         none
  */
float TargetCurve_CalcAtTime(const TargetCurve_t *obj, float time_s)
{
    return TargetCurve_CalcStateAtTime(obj, time_s).position;
}

/**
  * @brief          按周期更新并返回目标曲线位置输出
  * @retval         none
  */
float TargetCurve_Update(TargetCurve_t *obj, float dt_s)
{
    return TargetCurve_UpdateState(obj, dt_s).position;
}

/**
  * @brief          获取目标曲线当前位置输出
  * @retval         none
  */
float TargetCurve_GetOutput(const TargetCurve_t *obj)
{
    if (obj == 0)
    {
        return 0.0f;
    }

    return obj->state.position;
}

/**
  * @brief          获取目标曲线当前位置输出
  * @retval         none
  */
float TargetCurve_GetVelocity(const TargetCurve_t *obj)
{
    if (obj == 0)
    {
        return 0.0f;
    }

    return obj->state.velocity;
}

/**
  * @brief          获取目标曲线当前加速度输出
  * @retval         none
  */
float TargetCurve_GetAcceleration(const TargetCurve_t *obj)
{
    if (obj == 0)
    {
        return 0.0f;
    }

    return obj->state.acceleration;
}
