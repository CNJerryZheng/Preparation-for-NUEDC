/**
 * @file        bsp_timer.h
 * @author      JerryZheng
 * @brief       MSPA 底盘控制节拍 BSP 接口
 * @date        2026-07-21
 */

#pragma once

#include <stdbool.h>

/**
 * @brief 使能 TIMG6 的 10ms 控制节拍中断
 */
void BSP_Timer_Init(void);

/**
 * @brief 取走一次待处理的 10ms 节拍
 * @retval bool 是否有新的 10ms 节拍
 */
bool BSP_Timer_TakeChassisTick(void);
