/**
 * @file        mt6816_config.h
 * @author      JerryZheng
 * @brief       MT6816 六线编码器协议与机械零点配置
 * @date        2026-07-23
 */

#pragma once

/** @brief MT6816 PWM 每帧固定时钟数。 */
#define MT6816_PWM_FRAME_CLOCKS (4119U)

/** @brief PWM 帧开头固定的高电平时钟数。 */
#define MT6816_PWM_START_HIGH_CLOCKS (16U)

/** @brief PWM 帧角度数据的满量程计数。 */
#define MT6816_PWM_ANGLE_CODES_PER_REV (4096U)

/** @brief A/B 正交四倍频后的单圈计数，本项目编码器为 1000 线。 */
#define MT6816_AB_COUNTS_PER_REV (4000)

/** @brief 4MHz 捕获时允许的最短 PWM 周期，对应约 971.1Hz 输出。 */
#define MT6816_PWM_MIN_PERIOD_TICKS (3500U)

/** @brief 4MHz 捕获时允许的最长 PWM 周期，兼容约 485.6Hz 输出。 */
#define MT6816_PWM_MAX_PERIOD_TICKS (9000U)

/**
 * @brief 接受 Z 相校准时允许的 PWM 零位窗口
 * @note 128 个 12 位码约为 11.25°，用于排除电机干扰产生的伪 Z 脉冲。
 */
#define MT6816_Z_VALID_WINDOW_PWM_CODES (128U)

/**
 * @brief Yaw 机械零点对应的 PWM 12 位码值
 * @note 默认使用 MT6816 已烧录的 0°。若装配零点不一致，可改为实测码值。
 */
#define MT6816_YAW_ZERO_PWM_CODE (0U)

/**
 * @brief Pitch 机械零点对应的 PWM 12 位码值
 * @note 默认使用 MT6816 已烧录的 0°。若装配零点不一致，可改为实测码值。
 */
#define MT6816_PITCH_ZERO_PWM_CODE (0U)

/** @brief Yaw PWM 角度增长方向，和 A/B 正方向相反时改为 -1。 */
#define MT6816_YAW_PWM_DIRECTION_SIGN (1)

/** @brief Pitch PWM 角度增长方向，和 A/B 正方向相反时改为 -1。 */
#define MT6816_PITCH_PWM_DIRECTION_SIGN (1)

/**
 * @brief 是否要求两轴 PWM 均完成绝对位置同步后才使能驱动器
 * @note 保持为 1U 最安全；调试纯 A/B 时可临时改为 0U。
 */
#define MT6816_REQUIRE_PWM_SYNC_BEFORE_ENABLE (1U)
