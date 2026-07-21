/**
 * @file        chassis_task.c
 * @author      JerryZheng
 * @brief       Chassis periodic service task.
 * @date        2026-07-20
 */

#include "chassis_task.h"
#include "chassis_control.h"
#include "linetrack.h"

void CHASSIS_TaskInit(void)
{
}

void CHASSIS_TaskProcess(void)
{
    LINE_Result_t line = LINE_Process();
    CHASSIS_ControlUpdate(&line);
}
