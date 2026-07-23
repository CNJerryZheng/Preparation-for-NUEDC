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
    // 判断当前运动方向是否受到限位保护。
    if (speed_cps > WHEEL_SPEED_TARGET_LIMIT_CPS)
    {
        // 返回本次计算或查询得到的结果。
        return WHEEL_SPEED_TARGET_LIMIT_CPS;
    }
    // 判断当前运动方向是否受到限位保护。
    if (speed_cps < -WHEEL_SPEED_TARGET_LIMIT_CPS)
    {
        // 返回本次计算或查询得到的结果。
        return -WHEEL_SPEED_TARGET_LIMIT_CPS;
    }
    // 返回本次计算或查询得到的结果。
    return speed_cps;
}

/**
 * @brief 将浮点 PID 输出转换为电机有符号占空比
 * @param output PID 浮点输出
 * @return int16_t -100 至 100 的有符号占空比
 */
static int16_t WHEEL_OutputToDuty(float output)
{
    // 判断 output > 100.0f 是否成立，并选择对应处理分支。
    if (output > 100.0f)
    {
        // 返回本次计算或查询得到的结果。
        return 100;
    }
    // 判断 output < -100.0f 是否成立，并选择对应处理分支。
    if (output < -100.0f)
    {
        // 返回本次计算或查询得到的结果。
        return -100;
    }
    // 返回本次计算或查询得到的结果。
    return (int16_t)output;
}

/**
 * @brief 清除双轮速度滑动平均历史
 */
static void WHEEL_ResetSpeedFilter(void)
{
    // 遍历当前数据集合并逐项完成处理。
    for (uint8_t index = 0U;
        index < WHEEL_SPEED_AVERAGE_WINDOW_SIZE; ++index)
    {
        // 更新 s_left_delta_history[index] 对应的累计计数。
        s_left_delta_history[index] = 0.0f;
        // 更新 s_right_delta_history[index] 对应的累计计数。
        s_right_delta_history[index] = 0.0f;
    }

    // 更新 s_delta_history_index 对应的累计计数。
    s_delta_history_index = 0U;
    // 更新 s_delta_history_count 对应的累计计数。
    s_delta_history_count = 0U;
    // 更新 s_left_delta_sum 对应的本步骤的运行数据。
    s_left_delta_sum = 0.0f;
    // 更新 s_right_delta_sum 对应的本步骤的运行数据。
    s_right_delta_sum = 0.0f;
}

/**
 * @brief 根据目标速度计算维持转速所需的前馈占空比
 * @param target_cps 目标速度，单位为霍尔计数/秒
 * @return float 有符号前馈占空比百分数
 */
static float WHEEL_CalculateFeedforward(float target_cps)
{
    // 返回本次计算或查询得到的结果。
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
    // 定义本步骤需要的局部数据并完成初始化。
    float output;

    // 零速目标直接清除PID历史，避免下次启动继承旧积分。
    if (target_cps == 0.0f)
    {
        // 调用 PID_PositionReset，复位对应模块的历史状态。
        PID_PositionReset(pid);
        // 返回本次计算或查询得到的结果。
        return 0.0f;
    }

    // 前馈承担主要稳态占空比，PID只修正模型和负载误差。
    output = WHEEL_CalculateFeedforward(target_cps) +
        PID_PositionCalculate(pid, target_cps, feedback_cps);
    // 判断 output > WHEEL_SPEED_PID_OUTPUT_MAX 是否成立，并选择对应处理分支。
    if (output > WHEEL_SPEED_PID_OUTPUT_MAX)
    {
        // 返回本次计算或查询得到的结果。
        return WHEEL_SPEED_PID_OUTPUT_MAX;
    }
    // 判断 output < WHEEL_SPEED_PID_OUTPUT_MIN 是否成立，并选择对应处理分支。
    if (output < WHEEL_SPEED_PID_OUTPUT_MIN)
    {
        // 返回本次计算或查询得到的结果。
        return WHEEL_SPEED_PID_OUTPUT_MIN;
    }
    // 返回本次计算或查询得到的结果。
    return output;
}

/**
 * @brief 初始化双轮速度闭环控制器
 */
void WHEEL_SpeedControlInit(void)
{
    // 调用 PID_PositionInit，初始化对应模块或运行状态。
    PID_PositionInit(&s_left_pid,
        WHEEL_SPEED_PID_DEFAULT_KP,
        WHEEL_SPEED_PID_DEFAULT_KI,
        WHEEL_SPEED_PID_DEFAULT_KD,
        WHEEL_SPEED_CONTROL_PERIOD_S,
        WHEEL_SPEED_PID_OUTPUT_MIN,
        WHEEL_SPEED_PID_OUTPUT_MAX);
    // 调用 PID_PositionInit，初始化对应模块或运行状态。
    PID_PositionInit(&s_right_pid,
        WHEEL_SPEED_PID_DEFAULT_KP,
        WHEEL_SPEED_PID_DEFAULT_KI,
        WHEEL_SPEED_PID_DEFAULT_KD,
        WHEEL_SPEED_CONTROL_PERIOD_S,
        WHEEL_SPEED_PID_OUTPUT_MIN,
        WHEEL_SPEED_PID_OUTPUT_MAX);

    // 更新 s_telemetry.left_target_cps 对应的目标值。
    s_telemetry.left_target_cps = WHEEL_SPEED_DEFAULT_LEFT_TARGET_CPS;
    // 更新 s_telemetry.left_feedback_cps 对应的本步骤的运行数据。
    s_telemetry.left_feedback_cps = 0.0f;
    // 更新 s_telemetry.left_output 对应的本步骤的运行数据。
    s_telemetry.left_output = 0.0f;
    // 更新 s_telemetry.right_target_cps 对应的目标值。
    s_telemetry.right_target_cps = WHEEL_SPEED_DEFAULT_RIGHT_TARGET_CPS;
    // 更新 s_telemetry.right_feedback_cps 对应的本步骤的运行数据。
    s_telemetry.right_feedback_cps = 0.0f;
    // 更新 s_telemetry.right_output 对应的本步骤的运行数据。
    s_telemetry.right_output = 0.0f;
    // 更新 s_telemetry.kp 对应的本步骤的运行数据。
    s_telemetry.kp = WHEEL_SPEED_PID_DEFAULT_KP;
    // 更新 s_telemetry.ki 对应的本步骤的运行数据。
    s_telemetry.ki = WHEEL_SPEED_PID_DEFAULT_KI;
    // 更新 s_telemetry.kd 对应的本步骤的运行数据。
    s_telemetry.kd = WHEEL_SPEED_PID_DEFAULT_KD;
    // 更新 s_telemetry.feedforward_gain 对应的本步骤的运行数据。
    s_telemetry.feedforward_gain = WHEEL_SPEED_FEEDFORWARD_GAIN;
    // 更新 s_telemetry.left_native_delta 对应的本步骤的运行数据。
    s_telemetry.left_native_delta = 0.0f;
    // 更新 s_telemetry.right_native_delta 对应的本步骤的运行数据。
    s_telemetry.right_native_delta = 0.0f;
    // 更新 s_telemetry.elapsed_ticks 对应的时间状态。
    s_telemetry.elapsed_ticks = 0.0f;
    // 更新 s_telemetry.enabled 对应的有效或使能状态。
    s_telemetry.enabled = false;
    // 更新 s_telemetry.update_count 对应的累计计数。
    s_telemetry.update_count = 0U;
    // 调用 MG513X_GetHallCount，读取当前反馈或状态。
    s_last_left_count = MG513X_GetHallCount(MG513X_MOTOR_LEFT);
    // 调用 MG513X_GetHallCount，读取当前反馈或状态。
    s_last_right_count = MG513X_GetHallCount(MG513X_MOTOR_RIGHT);
    // 调用 WHEEL_ResetSpeedFilter，复位对应模块的历史状态。
    WHEEL_ResetSpeedFilter();
}

/**
 * @brief 设置双轮速度闭环使能状态
 * @param enable true 启用 PID，false 关闭 PID 并清零输出
 */
void WHEEL_SpeedControlSetEnabled(bool enable)
{
    // 每次切换使能都清空输出、PID和测速窗口，避免历史状态突变。
    s_telemetry.enabled = enable;
    // 更新 s_telemetry.left_output 对应的本步骤的运行数据。
    s_telemetry.left_output = 0.0f;
    // 更新 s_telemetry.right_output 对应的本步骤的运行数据。
    s_telemetry.right_output = 0.0f;
    // 调用 PID_PositionReset，复位对应模块的历史状态。
    PID_PositionReset(&s_left_pid);
    // 调用 PID_PositionReset，复位对应模块的历史状态。
    PID_PositionReset(&s_right_pid);
    // 调用 MG513X_GetHallCount，读取当前反馈或状态。
    s_last_left_count = MG513X_GetHallCount(MG513X_MOTOR_LEFT);
    // 调用 MG513X_GetHallCount，读取当前反馈或状态。
    s_last_right_count = MG513X_GetHallCount(MG513X_MOTOR_RIGHT);
    // 调用 WHEEL_ResetSpeedFilter，复位对应模块的历史状态。
    WHEEL_ResetSpeedFilter();
}

/**
 * @brief 查询双轮速度闭环使能状态
 * @retval bool 当前使能状态
 */
bool WHEEL_SpeedControlIsEnabled(void)
{
    // 返回本次计算或查询得到的结果。
    return s_telemetry.enabled;
}

/**
 * @brief 设置左右轮目标速度
 * @param left_cps 左轮目标速度，单位为霍尔计数/秒
 * @param right_cps 右轮目标速度，单位为霍尔计数/秒
 */
void WHEEL_SpeedControlSetTarget(float left_cps, float right_cps)
{
    // 调用 WHEEL_ClampTarget，完成当前步骤的业务处理。
    s_telemetry.left_target_cps = WHEEL_ClampTarget(left_cps);
    // 调用 WHEEL_ClampTarget，完成当前步骤的业务处理。
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
    // 双轮共用同一组在线参数，修改后清除历史误差和积分。
    PID_PositionSetGains(&s_left_pid, kp, ki, kd);
    // 调用 PID_PositionSetGains，更新或发送对应数据。
    PID_PositionSetGains(&s_right_pid, kp, ki, kd);
    // 调用 PID_PositionReset，复位对应模块的历史状态。
    PID_PositionReset(&s_left_pid);
    // 调用 PID_PositionReset，复位对应模块的历史状态。
    PID_PositionReset(&s_right_pid);
    // 更新 s_telemetry.kp 对应的本步骤的运行数据。
    s_telemetry.kp = kp;
    // 更新 s_telemetry.ki 对应的本步骤的运行数据。
    s_telemetry.ki = ki;
    // 更新 s_telemetry.kd 对应的本步骤的运行数据。
    s_telemetry.kd = kd;
}

/**
 * @brief 在线设置速度前馈系数
 * @param gain 前馈系数，单位为占空比百分数/(霍尔计数/秒)
 */
void WHEEL_SpeedControlSetFeedforward(float gain)
{
    // 限制在线前馈范围，防止误命令直接产生过大占空比。
    if (gain < 0.0f)
    {
        // 更新 gain 对应的本步骤的运行数据。
        gain = 0.0f;
    }
    // 前一条件不成立时继续判断当前条件。
    else if (gain > 0.2f)
    {
        // 更新 gain 对应的本步骤的运行数据。
        gain = 0.2f;
    }

    // 前馈改变后清除PID状态，避免旧积分与新前馈叠加。
    s_feedforward_gain = gain;
    // 更新 s_telemetry.feedforward_gain 对应的本步骤的运行数据。
    s_telemetry.feedforward_gain = gain;
    // 调用 PID_PositionReset，复位对应模块的历史状态。
    PID_PositionReset(&s_left_pid);
    // 调用 PID_PositionReset，复位对应模块的历史状态。
    PID_PositionReset(&s_right_pid);
}

/**
 * @brief 执行一次轮速测量和 PID 运算
 * @param elapsed_ticks 距离上次测量实际经过的 10ms 节拍数
 */
void WHEEL_SpeedControlUpdate(uint32_t elapsed_ticks)
{
    // 调用 MG513X_GetHallCount，读取当前反馈或状态。
    const int32_t left_count = MG513X_GetHallCount(MG513X_MOTOR_LEFT);
    // 调用 MG513X_GetHallCount，读取当前反馈或状态。
    const int32_t right_count = MG513X_GetHallCount(MG513X_MOTOR_RIGHT);
    // 通过无符号差值兼容32位霍尔累计计数自然回绕。
    const int32_t left_delta = (int32_t)((uint32_t)left_count -
        (uint32_t)s_last_left_count);
    // 更新 right_delta 对应的本步骤的运行数据。
    const int32_t right_delta = (int32_t)((uint32_t)right_count -
        (uint32_t)s_last_right_count);
    // 定义本步骤需要的局部数据并完成初始化。
    float left_delta_per_tick;
    // 定义本步骤需要的局部数据并完成初始化。
    float right_delta_per_tick;

    // 没有经过有效控制节拍时不更新测速和PID。
    if (elapsed_ticks == 0U)
    {
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 将本次总增量折算为单个10ms节拍的平均增量。
    left_delta_per_tick = (float)left_delta / (float)elapsed_ticks;
    // 更新 right_delta_per_tick 对应的本步骤的运行数据。
    right_delta_per_tick = (float)right_delta / (float)elapsed_ticks;

    // 更新 s_last_left_count 对应的累计计数。
    s_last_left_count = left_count;
    // 更新 s_last_right_count 对应的累计计数。
    s_last_right_count = right_count;
    // 更新 s_telemetry.left_native_delta 对应的本步骤的运行数据。
    s_telemetry.left_native_delta = (float)left_delta;
    // 更新 s_telemetry.right_native_delta 对应的本步骤的运行数据。
    s_telemetry.right_native_delta = (float)right_delta;
    // 更新 s_telemetry.elapsed_ticks 对应的时间状态。
    s_telemetry.elapsed_ticks = (float)elapsed_ticks;
    // 用定长环形窗口维护双轮霍尔增量滑动和。
    s_left_delta_sum -= s_left_delta_history[s_delta_history_index];
    // 更新 s_right_delta_sum 对应的本步骤的运行数据。
    s_right_delta_sum -= s_right_delta_history[s_delta_history_index];
    // 更新 s_left_delta_history[s_delta_history_index] 对应的累计计数。
    s_left_delta_history[s_delta_history_index] = left_delta_per_tick;
    // 更新 s_right_delta_history[s_delta_history_index] 对应的累计计数。
    s_right_delta_history[s_delta_history_index] = right_delta_per_tick;
    // 更新 s_left_delta_sum 对应的本步骤的运行数据。
    s_left_delta_sum += left_delta_per_tick;
    // 更新 s_right_delta_sum 对应的本步骤的运行数据。
    s_right_delta_sum += right_delta_per_tick;
    // 更新 s_delta_history_index 对应的累计计数。
    s_delta_history_index = (uint8_t)((s_delta_history_index + 1U) %
        WHEEL_SPEED_AVERAGE_WINDOW_SIZE);
    // 判断 s_delta_history_count < WHEEL_SPEED_AVERAGE_WINDOW_SIZE 是否成立，并选择对应处理分支。
    if (s_delta_history_count < WHEEL_SPEED_AVERAGE_WINDOW_SIZE)
    {
        // 更新对应的累计计数或遍历位置。
        ++s_delta_history_count;
    }

    // 滑动平均结果换算为统一的x1霍尔计数每秒，并修正反馈方向。
    s_telemetry.left_feedback_cps =
        (s_left_delta_sum / (float)s_delta_history_count /
            WHEEL_SPEED_CONTROL_PERIOD_S) *
        WHEEL_SPEED_LEFT_FEEDBACK_SIGN /
        (float)MG513X_GetHallDecodeMultiplier(MG513X_MOTOR_LEFT);
    // 调用 MG513X_GetHallDecodeMultiplier，读取当前反馈或状态。
    s_telemetry.right_feedback_cps =
        (s_right_delta_sum / (float)s_delta_history_count /
            WHEEL_SPEED_CONTROL_PERIOD_S) *
        WHEEL_SPEED_RIGHT_FEEDBACK_SIGN /
        (float)MG513X_GetHallDecodeMultiplier(MG513X_MOTOR_RIGHT);

    // 使能时执行双轮独立PID，失能时保持软件输出为零。
    if (s_telemetry.enabled)
    {
        // 调用 WHEEL_CalculateOutput，更新并处理对应业务数据。
        s_telemetry.left_output = WHEEL_CalculateOutput(
            s_telemetry.left_target_cps,
            s_telemetry.left_feedback_cps, &s_left_pid);
        // 调用 WHEEL_CalculateOutput，更新并处理对应业务数据。
        s_telemetry.right_output = WHEEL_CalculateOutput(
            s_telemetry.right_target_cps,
            s_telemetry.right_feedback_cps, &s_right_pid);
    }
    // 前述条件均不成立时执行备用处理。
    else
    {
        // 更新 s_telemetry.left_output 对应的本步骤的运行数据。
        s_telemetry.left_output = 0.0f;
        // 更新 s_telemetry.right_output 对应的本步骤的运行数据。
        s_telemetry.right_output = 0.0f;
    }

    // 更新对应的累计计数或遍历位置。
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
    // 检查相关输入、计数或对象状态是否有效。
    if (left_percent != 0)
    {
        *left_percent = WHEEL_OutputToDuty(s_telemetry.left_output);
    }
    // 检查相关输入、计数或对象状态是否有效。
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
    // 检查相关输入、计数或对象状态是否有效。
    if (telemetry != 0)
    {
        *telemetry = s_telemetry;
    }
}
