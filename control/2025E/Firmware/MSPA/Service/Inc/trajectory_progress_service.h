/**
 * @file        trajectory_progress_service.h
 * @author      JerryZheng
 * @brief       小车轨迹进度计算与上报服务接口
 * @date        2026-07-23
 */

#pragma once

#include <stdint.h>

/** @brief Live Watch：当前轨迹进度，范围0~10000。 */
extern volatile uint16_t g_trajectory_progress;
/** @brief Live Watch：已发送的轨迹进度帧数量。 */
extern volatile uint32_t g_trajectory_progress_frame_count;

/** @brief 初始化轨迹行程累计值和发送节拍。 */
void TRAJECTORY_ProgressServiceInit(void);

/**
 * @brief 清零当前周期进度
 * @note 检测到可靠的起终点标志时可调用，用于消除轮胎打滑造成的累计误差。
 */
void TRAJECTORY_ProgressReset(void);

/** @brief 累计霍尔行程，并按50Hz向MSPB发送进度帧。 */
void TRAJECTORY_ProgressServiceProcess(void);

