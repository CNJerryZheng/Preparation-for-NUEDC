/**
 * @file        gimbal_task.c
 * @author      JerryZheng
 * @brief       Gimbal periodic service task.
 * @date        2026-07-20
 */

#include "gimbal_task.h"
#include "gimbal_axis.h"
#include "wt901.h"

void GIMBAL_TaskInit(void)
{
    WT901_Init();
}

void GIMBAL_TaskProcess(void)
{
    while (WT901_AnalyzeData())
    {
    }
    GIMBAL_AxisUpdate();
}
