/**
 * @file        line_control.c
 * @author      JerryZheng
 * @brief       八路循迹差速控制算法实现
 * @date        2026-07-22
 */

#include "line_control.h"
#include "line_control_config.h"

/** @brief 上一帧左轮速度目标，用于异常状态保持和速度平滑。 */
static float s_last_left_cps = 0.0f;
/** @brief 上一帧右轮速度目标，用于异常状态保持和速度平滑。 */
static float s_last_right_cps = 0.0f;
/** @brief 上一次有效黑线位置，用于 PD 微分计算和异常状态保持。 */
static float s_last_error = 0.0f;

/**
 * @brief 将循迹速度目标限制在配置范围内
 * @param speed_cps 原始有符号速度目标
 * @return float 限幅后的有符号速度目标
 */
static float LINE_ControlClamp(float speed_cps)
{
    if (speed_cps > LINE_CONTROL_MAX_SPEED_CPS)
    {
        return LINE_CONTROL_MAX_SPEED_CPS;
    }
    if (speed_cps < -LINE_CONTROL_MAX_SPEED_CPS)
    {
        return -LINE_CONTROL_MAX_SPEED_CPS;
    }
    return speed_cps;
}

/**
 * @brief 计算浮点数的绝对值
 * @param value 原始浮点数
 * @return float 非负绝对值
 */
static float LINE_ControlAbs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 保证正常循迹时车轮保持最低前进速度
 * @param speed_cps 已限幅的速度目标
 * @return float 处理后的前进速度目标
 */
static float LINE_ControlKeepForward(float speed_cps)
{
    if (speed_cps < LINE_CONTROL_MIN_FORWARD_SPEED_CPS)
    {
        return LINE_CONTROL_MIN_FORWARD_SPEED_CPS;
    }
    return speed_cps;
}

/**
 * @brief 根据位置误差计算正常循迹时的 PD 差速输出
 * @param error 当前黑线位置误差，负数为左侧，正数为右侧
 * @param left_cps 左轮目标速度输出地址
 * @param right_cps 右轮目标速度输出地址
 */
static void LINE_ControlCalculatePd(float error,
    float *left_cps, float *right_cps)
{
    // 偏差越大越主动降低基础速度，为急弯保留更多转向能力。
    float base_speed = LINE_CONTROL_BASE_SPEED_CPS -
        (LINE_ControlAbs(error) * LINE_CONTROL_TURN_SLOWDOWN_CPS);
    // 比例项决定转向强度，微分项抑制误差快速变化造成的过冲。
    const float derivative = error - s_last_error;
    const float correction = (LINE_CONTROL_KP_CPS * error) +
        (LINE_CONTROL_KD_CPS * derivative);

    // 正常循迹阶段不允许基础速度低于最低前进速度。
    if (base_speed < LINE_CONTROL_MIN_FORWARD_SPEED_CPS)
    {
        base_speed = LINE_CONTROL_MIN_FORWARD_SPEED_CPS;
    }

    // 左右轮使用相反修正量形成差速转向。
    *left_cps = LINE_ControlKeepForward(
        LINE_ControlClamp(base_speed + correction));
    *right_cps = LINE_ControlKeepForward(
        LINE_ControlClamp(base_speed - correction));
    s_last_error = error;
}

/**
 * @brief 将目标速度以限定步长平滑变化
 * @param current_cps 当前速度目标
 * @param target_cps 最终速度目标
 * @param elapsed_ticks 实际经过的 10ms 节拍数
 * @return float 本次控制后的速度目标
 */
static float LINE_ControlSlew(float current_cps,
    float target_cps, uint32_t elapsed_ticks)
{
    const float maximum_step = LINE_CONTROL_SPEED_STEP_CPS_PER_10MS *
        (float)elapsed_ticks;

    if (target_cps > current_cps + maximum_step)
    {
        return current_cps + maximum_step;
    }
    if (target_cps < current_cps - maximum_step)
    {
        return current_cps - maximum_step;
    }
    return target_cps;
}

/**
 * @brief 清除循迹控制的历史输出
 */
void LINE_ControlReset(void)
{
    s_last_left_cps = 0.0f;
    s_last_right_cps = 0.0f;
    s_last_error = 0.0f;
}

/**
 * @brief 根据八路循迹结果计算左右轮闭环速度目标
 * @param line 当前循迹结果
 * @param elapsed_ticks 距离上次计算实际经过的 10ms 节拍数
 * @param left_cps 左轮有符号速度目标输出地址
 * @param right_cps 右轮有符号速度目标输出地址
 */
void LINE_ControlCalculate(const LINE_Result_t *line, uint32_t elapsed_ticks,
    float *left_cps, float *right_cps)
{
    float target_left = LINE_CONTROL_BASE_SPEED_CPS;
    float target_right = LINE_CONTROL_BASE_SPEED_CPS;

    // 输出地址无效或没有控制节拍时保持调用方原值。
    if ((left_cps == 0) || (right_cps == 0) || (elapsed_ticks == 0U))
    {
        return;
    }

    // 正常识别黑线时按连续位置误差执行PD差速。
    if ((line != 0) && (line->state == State_OK))
    {
        float position = line->position;
#if LINE_CONTROL_X1_ON_LEFT == 0U
        position = -position;
#endif
        LINE_ControlCalculatePd(position, &target_left, &target_right);
    }
    else if ((line != 0) && (line->state == State_Lose_Left))
    {
        // 从左侧丢线时按最后方向原地搜索，外侧轮保持较高速度。
        target_left = LINE_CONTROL_LOST_INNER_SPEED_CPS;
        target_right = LINE_CONTROL_LOST_OUTER_SPEED_CPS;
    }
    else if ((line != 0) && (line->state == State_Lose_Right))
    {
        // 从右侧丢线时采用与左侧丢线镜像的搜索动作。
        target_left = LINE_CONTROL_LOST_OUTER_SPEED_CPS;
        target_right = LINE_CONTROL_LOST_INNER_SPEED_CPS;
    }
    else if ((line != 0) && ((line->state == State_Discontinuous) ||
        (line->state == State_Lose_Unknown) ||
        (line->state == State_Unknown)))
    {
        // 状态暂时不可信时沿用最近有效误差，避免突然停车或随机转向。
        LINE_ControlCalculatePd(s_last_error, &target_left, &target_right);
    }

    // 最终目标先做绝对限幅，再按控制周期执行速度斜坡。
    target_left = LINE_ControlClamp(target_left);
    target_right = LINE_ControlClamp(target_right);
    s_last_left_cps = LINE_ControlSlew(
        s_last_left_cps, target_left, elapsed_ticks);
    s_last_right_cps = LINE_ControlSlew(
        s_last_right_cps, target_right, elapsed_ticks);
    *left_cps = s_last_left_cps;
    *right_cps = s_last_right_cps;
}
