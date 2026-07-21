/**
 * @file        chassis_control.c
 * @author      JerryZheng
 * @brief       底盘控制算法层
 * @date        2026-07-21
 */

#include "chassis_control.h"

/**
 * @brief 根据循迹结果更新底盘控制算法
 * @param line 当前循迹结果
 */
void CHASSIS_ControlUpdate(const LINE_Result_t *line)
{
    /* 控制参数与电机执行策略尚未确定，暂不在算法层写寄存器。 */
    (void)line;
}
