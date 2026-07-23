/**
 * @file        imu_follow_service.h
 * @author      JerryZheng
 * @brief       WT901 姿态跟随云台业务接口
 * @date        2026-07-23
 */

#pragma once

#include <stdint.h>

/** @brief Live Watch：WT901 跟随是否正在工作。 */
extern volatile uint8_t g_wt901_follow_active;

/** @brief Live Watch：解缠并滤波后的 Yaw 相对角度。 */
extern volatile float g_wt901_follow_yaw_deg;

/** @brief Live Watch：解缠并滤波后的 Pitch 相对角度。 */
extern volatile float g_wt901_follow_pitch_deg;

/** @brief Live Watch：距离最近 WT901 角度帧的时间。 */
extern volatile uint32_t g_wt901_follow_age_ms;

/**
 * @brief 初始化 WT901 姿态跟随业务
 */
void IMU_FollowServiceInit(void);

/**
 * @brief 根据最新 WT901 角度更新两轴相对位置目标
 * @param elapsed_ms 距离上次处理实际经过的毫秒数
 */
void IMU_FollowServiceProcess(uint32_t elapsed_ms);
