/**
 * @file        gimbal_task.c
 * @author      JerryZheng
 * @brief       云台周期业务服务
 * @date        2026-07-21
 */

#include "gimbal_task.h"
#include "gimbal_axis.h"
#include "wt901.h"
#include "bsp_timer.h"

/**
 * @brief 初始化云台周期业务
 */
void GIMBAL_TaskInit(void)
{
    if (WT901_Init() == WT901_OK)
    {
        (void)WT901_StartReceive();
    }
}

/**
 * @brief 在 1ms 节拍到达时处理姿态数据与云台控制业务
 */
void GIMBAL_TaskProcess(void)
{
    while (WT901_AnalyzeData())
    {
    }

    if (!BSP_Timer_TakeGimbalTick())
    {
        return;
    }
    GIMBAL_AxisUpdate();
}
