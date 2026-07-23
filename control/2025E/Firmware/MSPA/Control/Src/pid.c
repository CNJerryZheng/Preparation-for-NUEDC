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

    if ((pid == 0) || (pid->sample_time_s <= 0.0f))
    {
        return 0.0f;
    }

    if (pid->initialized)
    {
        derivative = (error - pid->last_error) / pid->sample_time_s;
    }

    candidate_integral = pid->integral + (error * pid->sample_time_s);
    candidate_integral = PID_Clamp(candidate_integral,
        -pid->integral_limit, pid->integral_limit);

    output = (pid->kp * error) + (pid->ki * candidate_integral) +
        (pid->kd * derivative);
    if (!(((output > pid->output_max) && (error > 0.0f)) ||
            ((output < pid->output_min) && (error < 0.0f))))
    {
        pid->integral = candidate_integral;
    }

    output = (pid->kp * error) + (pid->ki * pid->integral) +
        (pid->kd * derivative);
    output = PID_Clamp(output, pid->output_min, pid->output_max);
    pid->last_error = error;
    pid->initialized = true;
    return output;
}
