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
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
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
    if (pid == 0)
    {
        return;
    }

    // 初始化参数边界后统一复位动态状态，保证控制器从确定状态启动。
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->sample_time_s = sample_time_s;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_limit = 100000.0f;
    PID_PositionReset(pid);
}

/**
 * @brief 清除 PID 的积分和历史误差
 * @param pid PID 控制器地址
 */
void PID_PositionReset(PID_Position_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    // 下一次计算视为首个采样，不使用已经失效的误差历史计算微分项。
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
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
    if (pid == 0)
    {
        return;
    }

    // 在线改参只替换增益；是否清空积分由上层根据模式切换决定。
    pid->kp = kp;
    pid->ki = ki;
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
    float derivative = 0.0f;
    float candidate_integral;
    float output;
    const float error = target - feedback;

    // 控制器地址或采样周期无效时输出安全零值。
    if ((pid == 0) || (pid->sample_time_s <= 0.0f))
    {
        return 0.0f;
    }

    // 第一拍没有历史误差，因此微分项保持为零以避免启动冲击。
    if (pid->initialized)
    {
        derivative = (error - pid->last_error) / pid->sample_time_s;
    }

    // 先计算候选积分并限幅，防止长时间误差造成积分数值失控。
    candidate_integral = pid->integral + (error * pid->sample_time_s);
    candidate_integral = PID_Clamp(candidate_integral,
        -pid->integral_limit, pid->integral_limit);

    // 输出继续朝饱和方向增长时暂停积分，实现条件式抗积分饱和。
    output = (pid->kp * error) + (pid->ki * candidate_integral) +
        (pid->kd * derivative);
    if (!(((output > pid->output_max) && (error > 0.0f)) ||
            ((output < pid->output_min) && (error < 0.0f))))
    {
        pid->integral = candidate_integral;
    }

    // 使用最终积分状态重新计算并限制控制器输出。
    output = (pid->kp * error) + (pid->ki * pid->integral) +
        (pid->kd * derivative);
    output = PID_Clamp(output, pid->output_min, pid->output_max);
    // 保留本次误差，下一周期才能计算真实的误差变化率。
    pid->last_error = error;
    pid->initialized = true;
    return output;
}
