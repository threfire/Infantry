#include "adrc.h"

#include <math.h>

/*
 * ======================== 使用示例（注释版） ========================
 *
 * 下面以“摩擦轮速度环 ADRC，输出电流命令”为例说明如何使用。
 *
 * 1. 定义一个 ADRC 控制器实例
 *
 *    static ADRC_Controller g_friction_wheel_adrc;
 *
 * 2. 准备电机模型参数
 *
 *    ADRC_MotorModel motor = {
 *        .torque_constant = 0.0021f,
 *        .inertia = 1.6e-5f
 *    };
 *
 * 3. 生成一套基础配置
 *
 *    ADRC_Config cfg;
 *    ADRC_ConfigFromMotor(&cfg,
 *                         &motor,
 *                         0.001f,     // 控制周期 1 ms
 *                         0.03f,      // 期望响应时间 30 ms
 *                         6000.0f,    // 速度量程 6000 RPM
 *                         8.0f,       // 输出上限 8 A
 *                         300.0f);    // 输出变化率限制 300 A/s
 *
 *    cfg.observer_bandwidth_ratio = 4.0f;
 *    cfg.tracking_gain = 0.0f;
 *
 * 4. 初始化并复位
 *
 *    ADRC_Init(&g_friction_wheel_adrc, &cfg);
 *    ADRC_Reset(&g_friction_wheel_adrc, actual_speed_rpm, target_speed_rpm);
 *
 *    如果对象已经在稳定运转，例如摩擦轮一直开着，只是此时才切入 ADRC，
 *    更建议使用下面这种“热启动”方式，使 C 代码和 MATLAB 仿真文件里的
 *    start_from_steady_speed 逻辑保持一致：
 *
 *    hold_current_a = 1.6f;  // 当前维持稳速所需电流，仅示例
 *    ADRC_ResetWithState(&g_friction_wheel_adrc,
 *                        actual_speed_rpm,
 *                        target_speed_rpm,
 *                        hold_current_a,
 *                        -cfg.b0 * hold_current_a);
 *
 * 5. 在每个控制周期调用
 *
 *    current_cmd_a = ADRC_Update(&g_friction_wheel_adrc,
 *                                target_speed_rpm,
 *                                actual_speed_rpm);
 *
 * 6. 如果想观察负载扰动变化
 *
 *    disturb = ADRC_GetDisturbance(&g_friction_wheel_adrc);
 *
 * 7. 如果想在线调快或调慢响应
 *
 *    ADRC_SetBandwidth(&g_friction_wheel_adrc, 250.0f, 4.0f);
 *
 * 说明：
 * - 如果你已经通过实验辨识拿到了 b0，也可以直接写 cfg.b0，
 *   并把 cfg.motor 置为 0。
 * - 如果工程里启用了自动标定，可以先用自动标定得到配置，
 *   再调用 ADRC_Init。
 * - 当前 adrc.c 的 ESO 更新、fal 形式、输出变化率限制、输出限幅，
 *   已经与 friction_wheel_adrc_autotune.m 中的核心控制律保持一致。
 * - 之前 C 版只有“清零式复位”，和 MATLAB 常开工况的稳态起步不完全一致；
 *   现在通过 ADRC_ResetWithState 补齐了这部分差异。
 * - 如果你希望像 pid.c/h 那样接入工程，则优先使用：
 *   ADRC_init / ADRC_set_param / ADRC_update_fdb / ADRC_update_ref /
 *   ADRC_calc / ADRC_Calc / ADRC_clear 这一组接口。
 * ===================================================================
 */

/**
 * @brief 求浮点数绝对值
 */
static float adrc_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
 * @brief 通用限幅函数
 */
static float adrc_clamp(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        return max_value;
    }

    if(value < min_value)
    {
        return min_value;
    }

    return value;
}

/**
 * @brief fal 非线性函数
 *
 * 小误差区采用线性段，避免噪声被过度放大；
 * 大误差区采用幂函数，提高系统动态响应能力。
 */
static float adrc_fal(float error, float alpha, float delta)
{
    float abs_error;

    abs_error = adrc_absf(error);
    if(abs_error <= delta)
    {
        return error / powf(delta, 1.0f - alpha);
    }

    return powf(abs_error, alpha) * ((error >= 0.0f) ? 1.0f : -1.0f);
}

/**
 * @brief 解析配置中的 b0
 *
 * 优先级如下：
 * 1. 如果 config->b0 > 0，直接使用
 * 2. 否则尝试用 torque_constant / inertia 自动估算
 * 3. 都不满足则返回 0，表示无效
 */
static float adrc_resolve_b0(const ADRC_Config *config)
{
    if(config->b0 > 0.0f)
    {
        return config->b0;
    }

    if((config->motor != 0) &&
       (config->motor->torque_constant > 0.0f) &&
       (config->motor->inertia > 0.0f))
    {
        return config->motor->torque_constant / config->motor->inertia;
    }

    return 0.0f;
}

/**
 * @brief 把简化参数结构转换成底层通用配置
 */
static int adrc_fill_config_from_param(ADRC_Config *config, const adrc_param_t *param)
{
    if((config == 0) || (param == 0))
    {
        return ADRC_ERR_INVALID_ARG;
    }

    config->sample_time_s = param->sample_time_s;
    config->b0 = param->b0;
    config->controller_bandwidth = param->controller_bandwidth;
    config->observer_bandwidth_ratio = param->observer_bandwidth_ratio;
    config->tracking_gain = param->tracking_gain;
    config->output_min = -param->max_out;
    config->output_max = param->max_out;
    config->output_rate_limit = param->output_rate_limit;
    config->error_linear_zone = param->error_linear_zone;
    config->alpha1 = param->alpha1;
    config->alpha2 = param->alpha2;
    config->motor = 0;

    return ADRC_OK;
}

/**
 * @brief 同步对外可见状态，方便工程按 PID 风格读取
 */
static void adrc_sync_public_state(adrc_type_def *adrc)
{
    if(adrc == 0)
    {
        return;
    }

    adrc->out = ADRC_GetLastOutput(&adrc->core);
    adrc->disturbance = ADRC_GetDisturbance(&adrc->core);
    adrc->tracking_state = ADRC_GetTrackingState(&adrc->core);
}

/**
 * @brief 检查初始化参数是否合法
 */
static int adrc_validate_config(const ADRC_Config *config)
{
    float b0;

    if(config == 0)
    {
        return ADRC_ERR_INVALID_ARG;
    }

    if(config->sample_time_s <= 0.0f)
    {
        return ADRC_ERR_INVALID_PARAM;
    }

    if(config->controller_bandwidth <= 0.0f)
    {
        return ADRC_ERR_INVALID_PARAM;
    }

    if(config->output_max <= config->output_min)
    {
        return ADRC_ERR_INVALID_PARAM;
    }

    b0 = adrc_resolve_b0(config);
    if(b0 <= 0.0f)
    {
        return ADRC_ERR_INVALID_PARAM;
    }

    return ADRC_OK;
}

int ADRC_Init(ADRC_Controller *controller, const ADRC_Config *config)
{
    int status;
    float observer_ratio;

    if(controller == 0)
    {
        return ADRC_ERR_INVALID_ARG;
    }

    status = adrc_validate_config(config);
    if(status != ADRC_OK)
    {
        return status;
    }

    observer_ratio = (config->observer_bandwidth_ratio > 0.0f) ?
                     config->observer_bandwidth_ratio :
                     ADRC_DEFAULT_OBSERVER_RATIO;

    controller->sample_time_s = config->sample_time_s;
    controller->b0 = adrc_resolve_b0(config);
    controller->controller_bandwidth = config->controller_bandwidth;
    controller->observer_bandwidth_ratio = observer_ratio;
    controller->observer_bandwidth = config->controller_bandwidth * observer_ratio;
    controller->tracking_gain = (config->tracking_gain >= 0.0f) ? config->tracking_gain : 0.0f;
    controller->output_min = config->output_min;
    controller->output_max = config->output_max;
    controller->output_rate_limit = (config->output_rate_limit >= 0.0f) ? config->output_rate_limit : 0.0f;
    controller->error_linear_zone = (config->error_linear_zone > 0.0f) ?
                                    config->error_linear_zone :
                                    ADRC_DEFAULT_ERROR_LINEAR_ZONE;
    controller->alpha1 = ((config->alpha1 > 0.0f) && (config->alpha1 < 1.0f)) ?
                         config->alpha1 :
                         ADRC_DEFAULT_ALPHA1;
    controller->alpha2 = ((config->alpha2 > 0.0f) && (config->alpha2 < 1.0f)) ?
                         config->alpha2 :
                         ADRC_DEFAULT_ALPHA2;

    /* 一阶 ADRC 中，控制器等效比例增益通常直接取 wc。 */
    controller->kp = controller->controller_bandwidth;

    /* 二阶 ESO 增益。 */
    controller->beta1 = 2.0f * controller->observer_bandwidth;
    controller->beta2 = controller->observer_bandwidth * controller->observer_bandwidth;

    /* 清零内部状态。 */
    controller->z1 = 0.0f;
    controller->z2 = 0.0f;
    controller->tracking_state = 0.0f;
    controller->last_output = 0.0f;

    return ADRC_OK;
}

int ADRC_ConfigFromMotor(ADRC_Config *config,
                         const ADRC_MotorModel *motor,
                         float sample_time_s,
                         float response_time_s,
                         float feedback_range,
                         float output_limit,
                         float output_rate_limit)
{
    if((config == 0) || (motor == 0))
    {
        return ADRC_ERR_INVALID_ARG;
    }

    if((sample_time_s <= 0.0f) ||
       (response_time_s <= 0.0f) ||
       (feedback_range <= 0.0f) ||
       (output_limit <= 0.0f) ||
       (motor->torque_constant <= 0.0f) ||
       (motor->inertia <= 0.0f))
    {
        return ADRC_ERR_INVALID_PARAM;
    }

    config->sample_time_s = sample_time_s;
    config->b0 = 0.0f;
    config->controller_bandwidth = 5.0f / response_time_s;
    config->observer_bandwidth_ratio = ADRC_DEFAULT_OBSERVER_RATIO;
    config->tracking_gain = 0.0f;
    config->output_min = -output_limit;
    config->output_max = output_limit;
    config->output_rate_limit = (output_rate_limit >= 0.0f) ? output_rate_limit : 0.0f;
    config->error_linear_zone = feedback_range * 0.01f;
    config->alpha1 = ADRC_DEFAULT_ALPHA1;
    config->alpha2 = ADRC_DEFAULT_ALPHA2;
    config->motor = motor;

    if(config->error_linear_zone <= 0.0f)
    {
        config->error_linear_zone = ADRC_DEFAULT_ERROR_LINEAR_ZONE;
    }

    return ADRC_OK;
}

void ADRC_Reset(ADRC_Controller *controller, float measurement, float target)
{
    /*
     * 这是“冷启动”复位：
     * 1. 适合对象刚启动、输出尚未建立的场景
     * 2. z2 与 last_output 清零
     *
     * 如果对象已经在运行中，例如摩擦轮常开、只是切换控制模式，
     * 则建议改用 ADRC_ResetWithState，以减少进入闭环瞬间的假扰动。
     */
    ADRC_ResetWithState(controller, measurement, target, 0.0f, 0.0f);
}

void ADRC_ResetWithState(ADRC_Controller *controller,
                         float measurement,
                         float target,
                         float initial_output,
                         float initial_disturbance)
{
    if(controller == 0)
    {
        return;
    }

    /*
     * 这组初始化与 MATLAB 仿真中的“从稳态工作点开始”对应关系如下：
     * - z1 <- 当前测量值
     * - z2 <- 当前总扰动估计
     * - last_output <- 当前已在执行的控制输出
     *
     * 对一阶 ADRC，在稳态且 tracking_state = target 时，常有：
     * z2 ≈ -b0 * u
     * 因此如果你知道当前维持稳速的电流命令 initial_output，
     * 可以近似写成 initial_disturbance = -controller->b0 * initial_output。
     */
    controller->z1 = measurement;
    controller->z2 = initial_disturbance;
    controller->tracking_state = target;
    controller->last_output = adrc_clamp(initial_output,
                                         controller->output_min,
                                         controller->output_max);
}

float ADRC_Update(ADRC_Controller *controller, float target, float measurement)
{
    float observer_error;
    float track_error;
    float base_control;
    float control_output;
    float delta_output;

    if(controller == 0)
    {
        return 0.0f;
    }

    /* 第一步：目标跟踪器。 */
    if(controller->tracking_gain > 0.0f)
    {
        controller->tracking_state += controller->tracking_gain *
                                      (target - controller->tracking_state) *
                                      controller->sample_time_s;
    }
    else
    {
        controller->tracking_state = target;
    }

    /* 第二步：ESO 估计状态 z1 和总扰动 z2。 */
    observer_error = controller->z1 - measurement;

    controller->z1 += controller->sample_time_s *
                      (controller->z2 -
                       controller->beta1 * observer_error +
                       controller->b0 * controller->last_output);

    controller->z2 += controller->sample_time_s *
                      (-controller->beta2 *
                       adrc_fal(observer_error, controller->alpha2, controller->error_linear_zone));

    /* 第三步：非线性误差反馈 + 扰动补偿。 */
    track_error = controller->tracking_state - controller->z1;
    base_control = controller->kp *
                   adrc_fal(track_error, controller->alpha1, controller->error_linear_zone);
    control_output = (base_control - controller->z2) / controller->b0;

    /* 第四步：输出变化率限制。 */
    if(controller->output_rate_limit > 0.0f)
    {
        delta_output = control_output - controller->last_output;
        delta_output = adrc_clamp(delta_output,
                                  -controller->output_rate_limit * controller->sample_time_s,
                                  controller->output_rate_limit * controller->sample_time_s);
        control_output = controller->last_output + delta_output;
    }

    /* 第五步：输出限幅。 */
    control_output = adrc_clamp(control_output, controller->output_min, controller->output_max);
    controller->last_output = control_output;

    return control_output;
}

void ADRC_SetBandwidth(ADRC_Controller *controller,
                       float controller_bandwidth,
                       float observer_bandwidth_ratio)
{
    float ratio;

    if((controller == 0) || (controller_bandwidth <= 0.0f))
    {
        return;
    }

    ratio = (observer_bandwidth_ratio > 0.0f) ?
            observer_bandwidth_ratio :
            controller->observer_bandwidth_ratio;

    if(ratio <= 0.0f)
    {
        ratio = ADRC_DEFAULT_OBSERVER_RATIO;
    }

    controller->controller_bandwidth = controller_bandwidth;
    controller->observer_bandwidth_ratio = ratio;
    controller->observer_bandwidth = controller_bandwidth * ratio;
    controller->kp = controller_bandwidth;
    controller->beta1 = 2.0f * controller->observer_bandwidth;
    controller->beta2 = controller->observer_bandwidth * controller->observer_bandwidth;
}

void ADRC_SetOutputLimit(ADRC_Controller *controller, float output_min, float output_max)
{
    if((controller == 0) || (output_max <= output_min))
    {
        return;
    }

    controller->output_min = output_min;
    controller->output_max = output_max;
    controller->last_output = adrc_clamp(controller->last_output, output_min, output_max);
}

void ADRC_SetOutputRateLimit(ADRC_Controller *controller, float output_rate_limit)
{
    if(controller == 0)
    {
        return;
    }

    controller->output_rate_limit = (output_rate_limit >= 0.0f) ? output_rate_limit : 0.0f;
}

float ADRC_GetDisturbance(const ADRC_Controller *controller)
{
    if(controller == 0)
    {
        return 0.0f;
    }

    return controller->z2;
}

float ADRC_GetTrackingState(const ADRC_Controller *controller)
{
    if(controller == 0)
    {
        return 0.0f;
    }

    return controller->tracking_state;
}

float ADRC_GetLastOutput(const ADRC_Controller *controller)
{
    if(controller == 0)
    {
        return 0.0f;
    }

    return controller->last_output;
}

int ADRC_init(adrc_type_def *adrc, const adrc_param_t *param)
{
    int status;

    if((adrc == 0) || (param == 0))
    {
        return ADRC_ERR_INVALID_ARG;
    }

    status = adrc_fill_config_from_param(&adrc->config, param);
    if(status != ADRC_OK)
    {
        return status;
    }

    status = ADRC_Init(&adrc->core, &adrc->config);
    if(status != ADRC_OK)
    {
        return status;
    }

    /* 初始化后先把对外可见状态清零，行为尽量贴近 PID_init。 */
    adrc->set = 0.0f;
    adrc->fdb = 0.0f;
    adrc->out = 0.0f;
    adrc->last_out = 0.0f;
    adrc->delta_out = 0.0f;
    adrc->disturbance = 0.0f;
    adrc->tracking_state = 0.0f;
    return ADRC_OK;
}

int ADRC_set_param(adrc_type_def *adrc, const adrc_param_t *param)
{
    int status;
    float current_set;
    float current_fdb;
    float current_out;
    float current_disturbance;

    if((adrc == 0) || (param == 0))
    {
        return ADRC_ERR_INVALID_ARG;
    }

    /*
     * 在线改参数时，先保留当前工作点信息。
     * 这样重新初始化底层控制器后，还能尽量平滑地接回当前状态。
     */
    current_set = adrc->set;
    current_fdb = adrc->fdb;
    current_out = adrc->out;
    current_disturbance = adrc->disturbance;

    status = adrc_fill_config_from_param(&adrc->config, param);
    if(status != ADRC_OK)
    {
        return status;
    }

    status = ADRC_Init(&adrc->core, &adrc->config);
    if(status != ADRC_OK)
    {
        return status;
    }

    ADRC_ResetWithState(&adrc->core,
                        current_fdb,
                        current_set,
                        current_out,
                        current_disturbance);
    adrc_sync_public_state(adrc);
    adrc->set = current_set;
    adrc->fdb = current_fdb;
    adrc->last_out = adrc->out;
    adrc->delta_out = 0.0f;
    return ADRC_OK;
}

void ADRC_clear(adrc_type_def *adrc)
{
    if(adrc == 0)
    {
        return;
    }

    /* 行为上对齐 PID_clear：把输入、输出、内部状态都清空。 */
    ADRC_Reset(&adrc->core, 0.0f, 0.0f);
    adrc->set = 0.0f;
    adrc->fdb = 0.0f;
    adrc->out = 0.0f;
    adrc->last_out = 0.0f;
    adrc->delta_out = 0.0f;
    adrc->disturbance = 0.0f;
    adrc->tracking_state = 0.0f;
}

void ADRC_reset(adrc_type_def *adrc, float measurement, float target)
{
    if(adrc == 0)
    {
        return;
    }

    /* 冷启动复位：适合当前对象没有稳定输出、或希望从当前反馈重新起步。 */
    ADRC_Reset(&adrc->core, measurement, target);
    adrc->set = target;
    adrc->fdb = measurement;
    adrc->out = 0.0f;
    adrc->last_out = 0.0f;
    adrc->delta_out = 0.0f;
    adrc_sync_public_state(adrc);
}

void ADRC_hot_reset(adrc_type_def *adrc,
                    float measurement,
                    float target,
                    float initial_output)
{
    float initial_disturbance;

    if(adrc == 0)
    {
        return;
    }

    /*
     * 热启动复位：适合摩擦轮常开这类场景。
     * 对一阶 ADRC，可近似认为稳态下 z2 ≈ -b0 * u，
     * 因此这里用当前输出反推一个初始扰动估计。
     */
    initial_disturbance = -adrc->core.b0 * initial_output;
    ADRC_ResetWithState(&adrc->core,
                        measurement,
                        target,
                        initial_output,
                        initial_disturbance);

    adrc->set = target;
    adrc->fdb = measurement;
    adrc->out = initial_output;
    adrc->last_out = initial_output;
    adrc->delta_out = 0.0f;
    adrc_sync_public_state(adrc);
}

void ADRC_update_fdb(adrc_type_def *adrc, float fdb)
{
    if(adrc == 0)
    {
        return;
    }

    adrc->fdb = fdb;
}

void ADRC_update_ref(adrc_type_def *adrc, float set)
{
    if(adrc == 0)
    {
        return;
    }

    adrc->set = set;
}

float ADRC_calc(adrc_type_def *adrc)
{
    if(adrc == 0)
    {
        return 0.0f;
    }

    /*
     * 先保存上一拍输出，再计算当前输出，
     * 这样就能直接得到 delta_out，便于工程里做调试或限坡观察。
     */
    adrc->last_out = adrc->out;
    adrc->out = ADRC_Update(&adrc->core, adrc->set, adrc->fdb);
    adrc->delta_out = adrc->out - adrc->last_out;
    adrc_sync_public_state(adrc);
    return adrc->out;
}

float ADRC_Calc(adrc_type_def *adrc, float ref, float set)
{
    if(adrc == 0)
    {
        return 0.0f;
    }

    /* 调用顺序故意写成和 PID_Calc 一样，方便工程直接替换。 */
    ADRC_update_fdb(adrc, ref);
    ADRC_update_ref(adrc, set);
    return ADRC_calc(adrc);
}
