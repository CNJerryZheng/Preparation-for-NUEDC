/**
 * @file        gimbal_axis.c
 * @author      JerryZheng
 * @brief       云台轴控制算法层
 * @date        2026-07-21
 */

#include "gimbal_axis.h"

/**
 * @brief 根据当前姿态和目标值更新云台控制算法
 */
void GIMBAL_AxisUpdate(void)
{
    /* PID 参数和运动目标尚未确定，算法层暂不直接驱动步进器。 */
}
