/**
 * @file        pid.h
 * @author      JerryZheng
 * @brief       位置式 PID 控制器接口
 * @date        2026-07-22
 */

#pragma once

#include <stdbool.h>

/**
 * @brief 位置式 PID 控制器运行状态
 */
typedef struct
{
    float kp;              /**< 比例系数。 */
    float ki;              /**< 积分系数。 */
    float kd;              /**< 微分系数。 */
    float sample_time_s;   /**< 控制周期，单位为秒。 */
    float integral;        /**< 误差积分累计值。 */
    float last_error;      /**< 上一次误差。 */
    float output_min;      /**< 输出下限。 */
    float output_max;      /**< 输出上限。 */
    float integral_limit;  /**< 积分项绝对值限制。 */
    bool initialized;      /**< 是否已有上一帧误差。 */
} PID_Position_t;

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
    float sample_time_s, float output_min, float output_max);

/**
 * @brief 清除 PID 的积分和历史误差
 * @param pid PID 控制器地址
 */
void PID_PositionReset(PID_Position_t *pid);

/**
 * @brief 在线修改 PID 参数
 * @param pid PID 控制器地址
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_PositionSetGains(PID_Position_t *pid, float kp, float ki, float kd);

/**
 * @brief 执行一次位置式 PID 运算
 * @param pid PID 控制器地址
 * @param target 目标值
 * @param feedback 反馈值
 * @return float 本次 PID 输出
 */
float PID_PositionCalculate(PID_Position_t *pid,
    float target, float feedback);
