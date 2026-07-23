/**
 * @file        gimbal_task.h
 * @author      JerryZheng
 * @brief       云台周期业务服务接口
 * @date        2026-07-21
 */

#pragma once

#include <stdint.h>

/**
 * @brief 云台完整 Live Watch 诊断状态
 */
typedef struct
{
    float wt901_yaw_deg;          /**< WT901 当前 Yaw 角度。 */
    float wt901_pitch_deg;        /**< WT901 当前 Pitch 角度。 */
    uint32_t wt901_angle_frames;  /**< 已解析的 WT901 角度帧数量。 */
    uint8_t follow_active;        /**< WT901 跟随业务是否激活。 */
    uint8_t feedback_ready;       /**< 两轴编码器反馈是否全部就绪。 */
    float follow_yaw_deg;         /**< 滤波后的 Yaw 相对跟随角度。 */
    float follow_pitch_deg;       /**< 滤波后的 Pitch 相对跟随角度。 */
    uint32_t follow_age_ms;       /**< 距离最近角度帧的时间。 */
    uint8_t yaw_pwm_valid;        /**< Yaw 编码器 PWM 是否有效。 */
    uint8_t yaw_synchronized;     /**< Yaw 编码器是否完成绝对同步。 */
    uint32_t yaw_pwm_high_ticks;  /**< Yaw PWM 高电平捕获计数。 */
    uint32_t yaw_pwm_period_ticks;/**< Yaw PWM 周期捕获计数。 */
    uint32_t yaw_pwm_frames;      /**< Yaw 编码器 PWM 捕获帧数。 */
    uint8_t pitch_pwm_valid;      /**< Pitch 编码器 PWM 是否有效。 */
    uint8_t pitch_synchronized;   /**< Pitch 编码器是否完成绝对同步。 */
    uint32_t pitch_pwm_high_ticks;/**< Pitch PWM 高电平捕获计数。 */
    uint32_t pitch_pwm_period_ticks;/**< Pitch PWM 周期捕获计数。 */
    uint32_t pitch_pwm_frames;    /**< Pitch 编码器 PWM 捕获帧数。 */
    int32_t yaw_position;         /**< Yaw 编码器融合位置。 */
    int32_t pitch_position;       /**< Pitch 编码器融合位置。 */
    float yaw_target;             /**< Yaw 位置环目标计数。 */
    float pitch_target;           /**< Pitch 位置环目标计数。 */
    float yaw_step_hz;            /**< Yaw 当前有符号 STEP 频率。 */
    float pitch_step_hz;          /**< Pitch 当前有符号 STEP 频率。 */
} GIMBAL_DebugStatus_t;

/** @brief Live Watch：云台完整诊断状态，只需监视这一个变量。 */
extern volatile GIMBAL_DebugStatus_t g_gimbal_debug;

/** @brief 初始化云台周期业务状态 */
void GIMBAL_TaskInit(void);
/** @brief 执行一次非阻塞云台周期业务 */
void GIMBAL_TaskProcess(void);
