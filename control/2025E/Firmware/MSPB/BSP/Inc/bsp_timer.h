/**
 * @file        bsp_timer.h
 * @author      JerryZheng
 * @brief       MSPB 云台控制节拍 BSP 接口
 * @date        2026-07-21
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/** @brief TIMG6 已产生的 1ms 节拍次数，供 Live Watch 诊断使用 */
extern volatile uint32_t g_gimbal_tick_count;

/**
 * @brief 使能 TIMG6 的 1ms 控制节拍中断
 */
void BSP_Timer_Init(void);

/**
 * @brief 取走一次待处理的 1ms 节拍
 * @retval bool 是否有新的 1ms 节拍
 */
bool BSP_Timer_TakeGimbalTick(void);
