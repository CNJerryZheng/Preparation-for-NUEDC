/**
 * @file        trajectory_progress_config.h
 * @author      JerryZheng
 * @brief       小车轨迹进度上报参数配置
 * @date        2026-07-23
 */

#pragma once

/**
 * @brief 小车完成一次完整运动周期时两轮平均行驶的路程，单位m。
 * @note 默认值1.0m仅用于联调；上赛道前必须改为实测的完整一圈或完整8字路程。
 */
#define TRAJECTORY_PROGRESS_CYCLE_DISTANCE_M (1.0f)

/** @brief 轨迹进度满量程，对应协议中的100%。 */
#define TRAJECTORY_PROGRESS_FULL_SCALE (10000U)

/** @brief 每2个10ms底盘节拍发送一次，即50Hz。 */
#define TRAJECTORY_PROGRESS_SEND_INTERVAL_TICKS (2U)

