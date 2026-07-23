/**
 * @file        chassis_task.h
 * @author      JerryZheng
 * @brief       底盘周期业务服务接口
 * @date        2026-07-21
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "linetrack.h"

/** @brief Live Watch：最近一次八路循迹原始位图。 */
extern volatile uint8_t g_line_raw;
/** @brief Live Watch：最近一次循迹位置，负数靠 X1、正数靠 X8。 */
extern volatile float g_line_position;
/** @brief Live Watch：最近一次循迹状态。 */
extern volatile LINE_State_t g_line_state;
/** @brief Live Watch：当前左轮目标占空比百分数。 */
extern volatile int16_t g_line_left_duty_percent;
/** @brief Live Watch：当前右轮目标占空比百分数。 */
extern volatile int16_t g_line_right_duty_percent;
/** @brief Live Watch：循迹功能是否已由按键启动。 */
extern volatile bool g_line_follow_enabled;

/** @brief 初始化底盘周期业务状态 */
void CHASSIS_TaskInit(void);
/** @brief 执行一次非阻塞底盘周期业务 */
void CHASSIS_TaskProcess(void);
