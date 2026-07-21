/**
 * @file        chassis_control.h
 * @author      JerryZheng
 * @brief       底盘控制算法层接口
 * @date        2026-07-21
 */

#pragma once

#include "linetrack.h"

/**
 * @brief 根据循迹结果更新底盘控制算法
 * @param line 当前循迹结果
 */
void CHASSIS_ControlUpdate(const LINE_Result_t *line);
