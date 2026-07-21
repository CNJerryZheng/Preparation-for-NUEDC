/**
 * @file        chassis_task.c
 * @author      JerryZheng
 * @brief       底盘周期业务服务
 * @date        2026-07-21
 */

#include "chassis_task.h"
#include "bsp_timer.h"
#include "chassis_control.h"
#include "linetrack.h"

/**
 * @brief 初始化底盘周期业务
 */
void CHASSIS_TaskInit(void)
{
}

/**
 * @brief 在 10ms 节拍到达时处理循迹与底盘控制业务
 */
void CHASSIS_TaskProcess(void)
{
    if (!BSP_Timer_TakeChassisTick())
    {
        return;
    }
    LINE_Result_t line = LINE_Process();
    CHASSIS_ControlUpdate(&line);
}
