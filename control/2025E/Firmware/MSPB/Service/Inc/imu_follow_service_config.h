/**
 * @file        imu_follow_service_config.h
 * @author      JerryZheng
 * @brief       WT901 姿态跟随云台业务配置
 * @date        2026-07-23
 */

#pragma once

/** @brief 置 1U 启用 WT901 相对姿态跟随模式。 */
#define IMU_FOLLOW_ENABLE (1U)

/** @brief 置 1U 使 Yaw 电机跟随 WT901 yaw。 */
#define IMU_FOLLOW_YAW_ENABLE (1U)

/** @brief 置 1U 使 Pitch 电机跟随 WT901 pitch。 */
#define IMU_FOLLOW_PITCH_ENABLE (1U)

/** @brief Yaw 跟随方向，电机方向相反时改为 -1.0f。 */
#define IMU_FOLLOW_YAW_SIGN (1.0f)

/** @brief Pitch 跟随方向，电机方向相反时改为 -1.0f。 */
#define IMU_FOLLOW_PITCH_SIGN (1.0f)

/** @brief WT901 转角到 Yaw 云台转角的倍率，1.0f 表示一比一。 */
#define IMU_FOLLOW_YAW_SCALE (1.0f)

/** @brief WT901 转角到 Pitch 云台转角的倍率，1.0f 表示一比一。 */
#define IMU_FOLLOW_PITCH_SCALE (1.0f)

/** @brief 姿态目标一阶低通滤波时间常数，单位 ms。 */
#define IMU_FOLLOW_FILTER_TIME_CONSTANT_MS (30.0f)

/** @brief 目标角度变化小于该值时不刷新目标，用于抑制静止抖动。 */
#define IMU_FOLLOW_TARGET_DEADBAND_DEG (0.15f)

/** @brief 超过该时间没有新角度帧时保持当前位置，单位 ms。 */
#define IMU_FOLLOW_TIMEOUT_MS (100U)
