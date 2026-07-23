/**
 * @file        wheel_speed_control.h
 * @author      JerryZheng
 * @brief       双轮速度闭环控制接口
 * @date        2026-07-22
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 双轮速度闭环遥测数据
 */
typedef struct
{
    float left_target_cps;    /**< 左轮目标速度，单位为霍尔计数/秒。 */
    float left_feedback_cps;  /**< 左轮滤波反馈，单位为霍尔计数/秒。 */
    float left_output;        /**< 左轮 PID 输出，有符号占空比百分数。 */
    float right_target_cps;   /**< 右轮目标速度，单位为霍尔计数/秒。 */
    float right_feedback_cps; /**< 右轮滤波反馈，单位为霍尔计数/秒。 */
    float right_output;       /**< 右轮 PID 输出，有符号占空比百分数。 */
    float kp;                 /**< 当前比例系数。 */
    float ki;                 /**< 当前积分系数。 */
    float kd;                 /**< 当前微分系数。 */
    float feedforward_gain;   /**< 当前速度前馈系数。 */
    float left_native_delta;  /**< 左轮单周期硬件 QEI 原始增量，四倍频计数/10ms。 */
    float right_native_delta; /**< 右轮单周期 GPIO 原始增量，x1 计数/10ms。 */
    float elapsed_ticks;      /**< 本次测速实际跨过的 10ms 节拍数。 */
    bool enabled;             /**< 速度闭环是否启用。 */
    uint32_t update_count;    /**< 10ms 控制更新计数。 */
} WHEEL_SpeedTelemetry_t;

/**
 * @brief 初始化双轮速度闭环控制器
 */
void WHEEL_SpeedControlInit(void);

/**
 * @brief 设置双轮速度闭环使能状态
 * @param enable true 启用 PID，false 关闭 PID 并清零输出
 */
void WHEEL_SpeedControlSetEnabled(bool enable);

/**
 * @brief 查询双轮速度闭环使能状态
 * @retval bool 当前使能状态
 */
bool WHEEL_SpeedControlIsEnabled(void);

/**
 * @brief 设置左右轮目标速度
 * @param left_cps 左轮目标速度，单位为霍尔计数/秒
 * @param right_cps 右轮目标速度，单位为霍尔计数/秒
 */
void WHEEL_SpeedControlSetTarget(float left_cps, float right_cps);

/**
 * @brief 同时设置左右轮位置式 PID 参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void WHEEL_SpeedControlSetPid(float kp, float ki, float kd);

/**
 * @brief 在线设置速度前馈系数
 * @param gain 前馈系数，单位为占空比百分数/(霍尔计数/秒)
 */
void WHEEL_SpeedControlSetFeedforward(float gain);

/**
 * @brief 执行一次轮速测量和 PID 运算
 * @param elapsed_ticks 距离上次测量实际经过的 10ms 节拍数
 */
void WHEEL_SpeedControlUpdate(uint32_t elapsed_ticks);

/**
 * @brief 读取 PID 输出占空比
 * @param left_percent 左轮输出地址，可传入空指针
 * @param right_percent 右轮输出地址，可传入空指针
 */
void WHEEL_SpeedControlGetOutput(
    int16_t *left_percent, int16_t *right_percent);

/**
 * @brief 读取双轮速度闭环遥测数据
 * @param telemetry 遥测数据输出地址
 */
void WHEEL_SpeedControlGetTelemetry(WHEEL_SpeedTelemetry_t *telemetry);
