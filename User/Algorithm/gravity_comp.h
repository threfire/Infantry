#ifndef GRAVITY_COMP_H
#define GRAVITY_COMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OUTPUT_SCALE 0.72f

struct gimbal_control_t;

typedef struct
{
    /* 被补偿机构的总等效质量。 */
    float mass_kg;
    /* 质心相对旋转轴沿前向的偏移量。 */
    float com_forward_m;
    /* 质心相对旋转轴沿竖直方向的偏移量。 */
    float com_up_m;
    /* 重力加速度，传入非正值时回退到默认值。 */
    float gravity_mps2;
} gravity_comp_param_t;

typedef struct
{
    /* 当前使用的重力补偿参数。 */
    gravity_comp_param_t param;
    /* 力矩换算到输出命令的比例系数。 */
    float output_scale;
    /* 输出命令限幅。 */
    float output_limit;
} gravity_comp_t;

#define GRAVITY_COMP_DEFAULT_GRAVITY 9.80665f

/* 使用指定参数初始化重力补偿模块。 */
void gravity_comp_init(gravity_comp_t *comp, const gravity_comp_param_t *param);

/* 使用默认参数初始化重力补偿模块。 */
void gravity_comp_init_default(gravity_comp_t *comp);

/* 基于对象内部参数计算重力补偿力矩。 */
float gravity_comp_calc_torque(gravity_comp_t *comp, float pitch_rad);

/* 基于传入参数直接计算重力补偿力矩。 */
float gravity_comp_calc_torque_with_param(const gravity_comp_param_t *param, float pitch_rad);

/* 将重力补偿作为前馈量叠加到电机输出。 */
void gravity_comp_execute(struct gimbal_control_t *control);

#ifdef __cplusplus
}
#endif

#endif
