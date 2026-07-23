/**
 * @file        bsp_timer.h
 * @author      JerryZheng
 * @brief       MSPB 云台控制节拍 BSP 接口
 * @date        2026-07-21
 */

#pragma once

#include <stdint.h>

/** @brief TIMG6 已产生的 1ms 节拍次数，供 Live Watch 诊断使用 */
extern volatile uint32_t g_gimbal_tick_count;

/**
 * @brief 使能 TIMG6 的 1ms 控制节拍中断
 */
void BSP_Timer_Init(void);

/**
 * @brief 读取距离上次调用累计经过的1ms节拍数
 * @return uint32_t 累计节拍数，0表示没有新节拍
 * @note 使用单调累计计数，避免主循环短暂阻塞时丢失控制节拍。
 */
uint32_t BSP_Timer_TakeGimbalTicks(void);
