/**
 * @file        pid.c
 * @author      JerryZheng
 * @brief       位置式 PID 控制器实现
 * @date        2026-07-22
 */

#include "pid.h"

/**
 * @brief 将浮点数限制在指定范围内
 * @param value 原始数值
 * @param minimum 最小值
 * @param maximum 最大值
 * @return float 限幅后的数值
 */
static float PID_Clamp(float value, float minimum, float maximum)
{
    // 判断 value > maximum 是否成立，并选择对应处理分支。
    if (value > maximum)
    {
        // 返回本次计算或查询得到的结果。
        return maximum;
    }
    // 判断 value < minimum 是否成立，并选择对应处理分支。
    if (value < minimum)
    {
        // 返回本次计算或查询得到的结果。
        return minimum;
    }
    // 返回本次计算或查询得到的结果。
    return value;
}

/**
 * @brief 初始化位置式 PID 控制器
 * @param pid PID 控制器地址
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param sample_time_s 控制周期，单位为秒
 * @param output_min 输出下限
 * @param output_max 输出上限
 */
void PID_PositionInit(PID_Position_t *pid, float kp, float ki, float kd,
    float sample_time_s, float output_min, float output_max)
{
    // 检查相关输入、计数或对象状态是否有效。
    if (pid == 0)
    {
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 保存 pid->kp 对应的PID参数。
    pid->kp = kp;
    // 保存 pid->ki 对应的PID参数。
    pid->ki = ki;
    // 保存 pid->kd 对应的PID参数。
    pid->kd = kd;
    // 更新 pid->sample_time_s，保存当前语句的计算结果。
    pid->sample_time_s = sample_time_s;
    // 更新 pid->output_min 的控制输出值。
    pid->output_min = output_min;
    // 更新 pid->output_max 的控制输出值。
    pid->output_max = output_max;
    // 更新或清除积分状态，避免历史误差继续影响输出。
    pid->integral_limit = 100000.0f;
    // 调用 PID_PositionReset，复位对应模块的历史状态。
    PID_PositionReset(pid);
}

/**
 * @brief 清除 PID 的积分和历史误差
 * @param pid PID 控制器地址
 */
void PID_PositionReset(PID_Position_t *pid)
{
    // 检查相关输入、计数或对象状态是否有效。
    if (pid == 0)
    {
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 更新或清除积分状态，避免历史误差继续影响输出。
    pid->integral = 0.0f;
    // 保存当前误差，供下一周期计算微分项。
    pid->last_error = 0.0f;
    // 标记PID已完成首个周期，后续允许计算微分项。
    pid->initialized = false;
}

/**
 * @brief 在线修改 PID 参数
 * @param pid PID 控制器地址
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_PositionSetGains(PID_Position_t *pid, float kp, float ki, float kd)
{
    // 检查相关输入、计数或对象状态是否有效。
    if (pid == 0)
    {
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 保存 pid->kp 对应的PID参数。
    pid->kp = kp;
    // 保存 pid->ki 对应的PID参数。
    pid->ki = ki;
    // 保存 pid->kd 对应的PID参数。
    pid->kd = kd;
}

/**
 * @brief 执行一次位置式 PID 运算
 * @param pid PID 控制器地址
 * @param target 目标值
 * @param feedback 反馈值
 * @return float 本次 PID 输出
 */
float PID_PositionCalculate(PID_Position_t *pid,
    float target, float feedback)
{
    // 更新 derivative 对应的本步骤的运行数据。
    float derivative = 0.0f;
    // 定义本步骤需要的局部数据并完成初始化。
    float candidate_integral;
    // 定义本步骤需要的局部数据并完成初始化。
    float output;
    // 更新 error 对应的本步骤的运行数据。
    const float error = target - feedback;

    // 控制器地址或采样周期无效时输出安全零值。
    if ((pid == 0) || (pid->sample_time_s <= 0.0f))
    {
        // 返回本次计算或查询得到的结果。
        return 0.0f;
    }

    // 第一拍没有历史误差，因此微分项保持为零以避免启动冲击。
    if (pid->initialized)
    {
        // 更新 derivative 对应的本步骤的运行数据。
        derivative = (error - pid->last_error) / pid->sample_time_s;
    }

    // 先计算候选积分并限幅，防止长时间误差造成积分数值失控。
    candidate_integral = pid->integral + (error * pid->sample_time_s);
    // 调用 PID_Clamp，完成当前步骤的业务处理。
    candidate_integral = PID_Clamp(candidate_integral,
        -pid->integral_limit, pid->integral_limit);

    // 输出继续朝饱和方向增长时暂停积分，实现条件式抗积分饱和。
    output = (pid->kp * error) + (pid->ki * candidate_integral) +
        (pid->kd * derivative);
    // 根据当前控制误差选择对应处理方式。
    if (!(((output > pid->output_max) && (error > 0.0f)) ||
            ((output < pid->output_min) && (error < 0.0f))))
    {
        // 更新或清除积分状态，避免历史误差继续影响输出。
        pid->integral = candidate_integral;
    }

    // 使用最终积分状态重新计算并限制控制器输出。
    output = (pid->kp * error) + (pid->ki * pid->integral) +
        (pid->kd * derivative);
    // 调用 PID_Clamp，完成当前步骤的业务处理。
    output = PID_Clamp(output, pid->output_min, pid->output_max);
    // 保存当前误差，供下一周期计算微分项。
    pid->last_error = error;
    // 标记PID已完成首个周期，后续允许计算微分项。
    pid->initialized = true;
    // 返回本次计算或查询得到的结果。
    return output;
}
