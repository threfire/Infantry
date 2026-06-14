#include "gravity_comp.h"
#include "gimbal_task.h"

#include <math.h>
#include <string.h>

static float gravity_comp_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/* 使用外部给定参数初始化重力补偿模块。 */
void gravity_comp_init(gravity_comp_t *comp, const gravity_comp_param_t *param)
{
    if (comp == 0 || param == 0)
    {
        return;
    }

    memcpy(&comp->param, param, sizeof(comp->param));
    if (comp->param.gravity_mps2 <= 0.0f)
    {
        comp->param.gravity_mps2 = GRAVITY_COMP_DEFAULT_GRAVITY;
    }

    if (comp->output_scale <= 0.0f)
    {
        comp->output_scale = 1.0f;
    }

    if (comp->output_limit <= 0.0f)
    {
        comp->output_limit = T_MAX;
    }
}

/* 使用一组默认参数初始化模块，便于快速联调。 */
void gravity_comp_init_default(gravity_comp_t *comp)
{
    gravity_comp_param_t param;

    if (comp == 0)
    {
        return;
    }

    param.mass_kg = 1.5f;
    param.com_forward_m = 0.04f;
    param.com_up_m = 0.02f;
    param.gravity_mps2 = GRAVITY_COMP_DEFAULT_GRAVITY;

    comp->output_scale = OUTPUT_SCALE;
    comp->output_limit = T_MAX/10;

    gravity_comp_init(comp, &param);
}

/* 使用对象内部保存的参数计算当前俯仰角下的补偿力矩。 */
float gravity_comp_calc_torque(gravity_comp_t *comp, float pitch_rad)
{
    if (comp == 0)
    {
        return 0.0f;
    }

    return gravity_comp_calc_torque_with_param(&comp->param, pitch_rad);
}

/* 根据给定参数直接计算重力补偿力矩。 */
float gravity_comp_calc_torque_with_param(const gravity_comp_param_t *param, float pitch_rad)
{
    float gravity;
    float horizontal_distance;

    if (param == 0 || param->mass_kg <= 0.0f)
    {
        return 0.0f;
    }

    gravity = (param->gravity_mps2 > 0.0f) ? param->gravity_mps2 : GRAVITY_COMP_DEFAULT_GRAVITY;

    /*
     * 右手系 pitch 约定：
     * 1. pitch = 0 时，质心前向为 +x，竖直向上为 +z。
     * 2. 从 +y 方向看，抬头是顺时针，pitch_rad 为负。
     * 3. x_world = x*cos(theta) + z*sin(theta)。
     * 4. tau_comp = -m*g*x_world。
     */
    horizontal_distance = param->com_forward_m * cosf(pitch_rad) +
                          param->com_up_m * sinf(pitch_rad);

    return -param->mass_kg * gravity * horizontal_distance;
}

/* 将重力补偿作为前馈量叠加到控制输出。 */
void gravity_comp_execute(gimbal_control_t *control)
{
    gravity_comp_t *comp;
    gimbal_motor_t *motor;
    float gravity_torque;
    float gravity_feedforward;
    float command_output;

    if (control == 0)
    {
        return;
    }

//    if (control->gimbal_pitch_motor.mode == GIMBAL_MOTOR_RAW)
//    {
//        return;
//    }

    comp = &control->gimbal_pitch_gravity_comp;
    motor = &control->gimbal_pitch_motor;

    gravity_torque = gravity_comp_calc_torque(comp, motor->absolute_angle);
    gravity_feedforward = gravity_torque * comp->output_scale;

    motor->current_set += gravity_feedforward;
    command_output = gravity_comp_clamp(motor->current_set, -comp->output_limit, comp->output_limit);
    motor->output = command_output;
    motor->given_current = command_output;
}
