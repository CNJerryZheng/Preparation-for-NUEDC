/**
 * @file        bsp_timer.h
 * @author      JerryZheng
 * @brief       MSPA 底盘控制节拍 BSP 接口
 * @date        2026-07-21
 */

#pragma once

#include <stdint.h>

/**
 * @brief 使能 TIMG6 的 10ms 控制节拍中断
 */
void BSP_Timer_Init(void);

/**
 * @brief 读取距离上次调用累计经过的 10ms 节拍数
 * @return uint32_t 累计节拍数，0 表示没有新节拍
 * @note 使用单调累计计数，主循环短暂阻塞时不会把多个节拍合并丢失。
 */
uint32_t BSP_Timer_TakeChassisTicks(void);
