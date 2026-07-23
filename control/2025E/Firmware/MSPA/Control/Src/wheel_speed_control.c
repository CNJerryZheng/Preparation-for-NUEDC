/**
 * @file        wheel_speed_control.c
 * @author      JerryZheng
 * @brief       双轮速度闭环控制实现
 * @date        2026-07-22
 */

#include "wheel_speed_control.h"
#include "wheel_speed_control_config.h"
#include "mg513x.h"
#include "pid.h"

/** @brief 左轮位置式 PID 控制器。 */
static PID_Position_t s_left_pid;
/** @brief 右轮位置式 PID 控制器。 */
static PID_Position_t s_right_pid;
/** @brief 双轮速度闭环运行与遥测状态。 */
static WHEEL_SpeedTelemetry_t s_telemetry;
/** @brief 上一次左轮霍尔累计计数。 */
static int32_t s_last_left_count = 0;
/** @brief 上一次右轮霍尔累计计数。 */
static int32_t s_last_right_count = 0;
/** @brief 左轮最近若干次10ms霍尔计数增量。 */
static float s_left_delta_history[WHEEL_SPEED_AVERAGE_WINDOW_SIZE];
/** @brief 右轮最近若干次10ms霍尔计数增量。 */
static float s_right_delta_history[WHEEL_SPEED_AVERAGE_WINDOW_SIZE];
/** @brief 滑动平均环形缓冲区当前写入位置。 */
static uint8_t s_delta_history_index = 0U;
/** @brief 滑动平均当前已有的有效采样数量。 */
static uint8_t s_delta_history_count = 0U;
/** @brief 左轮滑动窗口霍尔增量总和。 */
static float s_left_delta_sum = 0.0f;
/** @brief 右轮滑动窗口霍尔增量总和。 */
static float s_right_delta_sum = 0.0f;
/** @brief 当前在线可调速度前馈系数。 */
static float s_feedforward_gain = WHEEL_SPEED_FEEDFORWARD_GAIN;

/**
 * @brief 将目标速度限制在安全调参范围内
 * @param speed_cps 原始目标速度，单位为霍尔计数/秒
 * @return float 限幅后的目标速度
 */
static float WHEEL_ClampTarget(float speed_cps)
{
    if (speed_cps > WHEEL_SPEED_TARGET_LIMIT_CPS)
    {
        return WHEEL_SPEED_TARGET_LIMIT_CPS;
    }
    if (speed_cps < -WHEEL_SPEED_TARGET_LIMIT_CPS)
    {
        return -WHEEL_SPEED_TARGET_LIMIT_CPS;
    }
    return speed_cps;
}

/**
 * @brief 将浮点 PID 输出转换为电机有符号占空比
 * @param output PID 浮点输出
 * @return int16_t -100 至 100 的有符号占空比
 */
static int16_t WHEEL_OutputToDuty(float output)
{
    if (output > 100.0f)
    {
        return 100;
    }
    if (output < -100.0f)
    {
        return -100;
    }
    return (int16_t)output;
}

/**
 * @brief 清除双轮速度滑动平均历史
 */
static void WHEEL_ResetSpeedFilter(void)
{
    for (uint8_t index = 0U;
        index < WHEEL_SPEED_AVERAGE_WINDOW_SIZE; ++index)
    {
        s_left_delta_history[index] = 0.0f;
        s_right_delta_history[index] = 0.0f;
    }

    s_delta_history_index = 0U;
    s_delta_history_count = 0U;
    s_left_delta_sum = 0.0f;
    s_right_delta_sum = 0.0f;
}

/**
 * @brief 根据目标速度计算维持转速所需的前馈占空比
 * @param target_cps 目标速度，单位为霍尔计数/秒
 * @return float 有符号前馈占空比百分数
 */
static float WHEEL_CalculateFeedforward(float target_cps)
{
    return target_cps * s_feedforward_gain;
}

/**
 * @brief 将 PID 修正量与前馈量合成为最终占空比
 * @param target_cps 目标速度，单位为霍尔计数/秒
 * @param feedback_cps 当前反馈速度，单位为霍尔计数/秒
 * @param pid PID 控制器地址
 * @return float 最终有符号占空比百分数
 */
static float WHEEL_CalculateOutput(float target_cps,
    float feedback_cps, PID_Position_t *pid)
{
    float output;

    if (target_cps == 0.0f)
    {
        PID_PositionReset(pid);
        return 0.0f;
    }

    output = WHEEL_CalculateFeedforward(target_cps) +
        PID_PositionCalculate(pid, target_cps, feedback_cps);
    if (output > WHEEL_SPEED_PID_OUTPUT_MAX)
    {
        return WHEEL_SPEED_PID_OUTPUT_MAX;
    }
    if (output < WHEEL_SPEED_PID_OUTPUT_MIN)
    {
        return WHEEL_SPEED_PID_OUTPUT_MIN;
    }
    return output;
}

/**
 * @brief 初始化双轮速度闭环控制器
 */
void WHEEL_SpeedControlInit(void)
{
    PID_PositionInit(&s_left_pid,
        WHEEL_SPEED_PID_DEFAULT_KP,
        WHEEL_SPEED_PID_DEFAULT_KI,
        WHEEL_SPEED_PID_DEFAULT_KD,
        WHEEL_SPEED_CONTROL_PERIOD_S,
        WHEEL_SPEED_PID_OUTPUT_MIN,
        WHEEL_SPEED_PID_OUTPUT_MAX);
    PID_PositionInit(&s_right_pid,
        WHEEL_SPEED_PID_DEFAULT_KP,
        WHEEL_SPEED_PID_DEFAULT_KI,
        WHEEL_SPEED_PID_DEFAULT_KD,
        WHEEL_SPEED_CONTROL_PERIOD_S,
        WHEEL_SPEED_PID_OUTPUT_MIN,
        WHEEL_SPEED_PID_OUTPUT_MAX);

    s_telemetry.left_target_cps = WHEEL_SPEED_DEFAULT_LEFT_TARGET_CPS;
    s_telemetry.left_feedback_cps = 0.0f;
    s_telemetry.left_output = 0.0f;
    s_telemetry.right_target_cps = WHEEL_SPEED_DEFAULT_RIGHT_TARGET_CPS;
    s_telemetry.right_feedback_cps = 0.0f;
    s_telemetry.right_output = 0.0f;
    s_telemetry.kp = WHEEL_SPEED_PID_DEFAULT_KP;
    s_telemetry.ki = WHEEL_SPEED_PID_DEFAULT_KI;
    s_telemetry.kd = WHEEL_SPEED_PID_DEFAULT_KD;
    s_telemetry.feedforward_gain = WHEEL_SPEED_FEEDFORWARD_GAIN;
    s_telemetry.left_native_delta = 0.0f;
    s_telemetry.right_native_delta = 0.0f;
    s_telemetry.elapsed_ticks = 0.0f;
    s_telemetry.enabled = false;
    s_telemetry.update_count = 0U;
    s_last_left_count = MG513X_GetHallCount(MG513X_MOTOR_LEFT);
    s_last_right_count = MG513X_GetHallCount(MG513X_MOTOR_RIGHT);
    WHEEL_ResetSpeedFilter();
}

/**
 * @brief 设置双轮速度闭环使能状态
 * @param enable true 启用 PID，false 关闭 PID 并清零输出
 */
void WHEEL_SpeedControlSetEnabled(bool enable)
{
    s_telemetry.enabled = enable;
    s_telemetry.left_output = 0.0f;
    s_telemetry.right_output = 0.0f;
    PID_PositionReset(&s_left_pid);
    PID_PositionReset(&s_right_pid);
    s_last_left_count = MG513X_GetHallCount(MG513X_MOTOR_LEFT);
    s_last_right_count = MG513X_GetHallCount(MG513X_MOTOR_RIGHT);
    WHEEL_ResetSpeedFilter();
}

/**
 * @brief 查询双轮速度闭环使能状态
 * @retval bool 当前使能状态
 */
bool WHEEL_SpeedControlIsEnabled(void)
{
    return s_telemetry.enabled;
}

/**
 * @brief 设置左右轮目标速度
 * @param left_cps 左轮目标速度，单位为霍尔计数/秒
 * @param right_cps 右轮目标速度，单位为霍尔计数/秒
 */
void WHEEL_SpeedControlSetTarget(float left_cps, float right_cps)
{
    s_telemetry.left_target_cps = WHEEL_ClampTarget(left_cps);
    s_telemetry.right_target_cps = WHEEL_ClampTarget(right_cps);
}

/**
 * @brief 同时设置左右轮位置式 PID 参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void WHEEL_SpeedControlSetPid(float kp, float ki, float kd)
{
    PID_PositionSetGains(&s_left_pid, kp, ki, kd);
    PID_PositionSetGains(&s_right_pid, kp, ki, kd);
    PID_PositionReset(&s_left_pid);
    PID_PositionReset(&s_right_pid);
    s_telemetry.kp = kp;
    s_telemetry.ki = ki;
    s_telemetry.kd = kd;
}

/**
 * @brief 在线设置速度前馈系数
 * @param gain 前馈系数，单位为占空比百分数/(霍尔计数/秒)
 */
void WHEEL_SpeedControlSetFeedforward(float gain)
{
    if (gain < 0.0f)
    {
        gain = 0.0f;
    }
    else if (gain > 0.2f)
    {
        gain = 0.2f;
    }

    s_feedforward_gain = gain;
    s_telemetry.feedforward_gain = gain;
    PID_PositionReset(&s_left_pid);
    PID_PositionReset(&s_right_pid);
}

/**
 * @brief 执行一次轮速测量和 PID 运算
 * @param elapsed_ticks 距离上次测量实际经过的 10ms 节拍数
 */
void WHEEL_SpeedControlUpdate(uint32_t elapsed_ticks)
{
    const int32_t left_count = MG513X_GetHallCount(MG513X_MOTOR_LEFT);
    const int32_t right_count = MG513X_GetHallCount(MG513X_MOTOR_RIGHT);
    const int32_t left_delta = (int32_t)((uint32_t)left_count -
        (uint32_t)s_last_left_count);
    const int32_t right_delta = (int32_t)((uint32_t)right_count -
        (uint32_t)s_last_right_count);
    float left_delta_per_tick;
    float right_delta_per_tick;

    if (elapsed_ticks == 0U)
    {
        return;
    }

    left_delta_per_tick = (float)left_delta / (float)elapsed_ticks;
    right_delta_per_tick = (float)right_delta / (float)elapsed_ticks;

    s_last_left_count = left_count;
    s_last_right_count = right_count;
    s_telemetry.left_native_delta = (float)left_delta;
    s_telemetry.right_native_delta = (float)right_delta;
    s_telemetry.elapsed_ticks = (float)elapsed_ticks;
    s_left_delta_sum -= s_left_delta_history[s_delta_history_index];
    s_right_delta_sum -= s_right_delta_history[s_delta_history_index];
    s_left_delta_history[s_delta_history_index] = left_delta_per_tick;
    s_right_delta_history[s_delta_history_index] = right_delta_per_tick;
    s_left_delta_sum += left_delta_per_tick;
    s_right_delta_sum += right_delta_per_tick;
    s_delta_history_index = (uint8_t)((s_delta_history_index + 1U) %
        WHEEL_SPEED_AVERAGE_WINDOW_SIZE);
    if (s_delta_history_count < WHEEL_SPEED_AVERAGE_WINDOW_SIZE)
    {
        ++s_delta_history_count;
    }

    s_telemetry.left_feedback_cps =
        (s_left_delta_sum / (float)s_delta_history_count /
            WHEEL_SPEED_CONTROL_PERIOD_S) *
        WHEEL_SPEED_LEFT_FEEDBACK_SIGN /
        (float)MG513X_GetHallDecodeMultiplier(MG513X_MOTOR_LEFT);
    s_telemetry.right_feedback_cps =
        (s_right_delta_sum / (float)s_delta_history_count /
            WHEEL_SPEED_CONTROL_PERIOD_S) *
        WHEEL_SPEED_RIGHT_FEEDBACK_SIGN /
        (float)MG513X_GetHallDecodeMultiplier(MG513X_MOTOR_RIGHT);

    if (s_telemetry.enabled)
    {
        s_telemetry.left_output = WHEEL_CalculateOutput(
            s_telemetry.left_target_cps,
            s_telemetry.left_feedback_cps, &s_left_pid);
        s_telemetry.right_output = WHEEL_CalculateOutput(
            s_telemetry.right_target_cps,
            s_telemetry.right_feedback_cps, &s_right_pid);
    }
    else
    {
        s_telemetry.left_output = 0.0f;
        s_telemetry.right_output = 0.0f;
    }

    ++s_telemetry.update_count;
}

/**
 * @brief 读取 PID 输出占空比
 * @param left_percent 左轮输出地址，可传入空指针
 * @param right_percent 右轮输出地址，可传入空指针
 */
void WHEEL_SpeedControlGetOutput(
    int16_t *left_percent, int16_t *right_percent)
{
    if (left_percent != 0)
    {
        *left_percent = WHEEL_OutputToDuty(s_telemetry.left_output);
    }
    if (right_percent != 0)
    {
        *right_percent = WHEEL_OutputToDuty(s_telemetry.right_output);
    }
}

/**
 * @brief 读取双轮速度闭环遥测数据
 * @param telemetry 遥测数据输出地址
 */
void WHEEL_SpeedControlGetTelemetry(WHEEL_SpeedTelemetry_t *telemetry)
{
    if (telemetry != 0)
    {
        *telemetry = s_telemetry;
    }
}
