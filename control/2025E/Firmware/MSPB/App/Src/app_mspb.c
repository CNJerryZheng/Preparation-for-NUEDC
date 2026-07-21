/**
 * @file        app_mspb.c
 * @author      JerryZheng
 * @brief       MSPB 云台应用调度层
 * @date        2026-07-21
 */

#include "app_mspb.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include "gimbal_task.h"
#include "communication_service.h"

/**
 * @brief 初始化云台应用所需的 BSP 与服务层
 */
void APP_MSPB_Init(void)
{
    BSP_UART_Init();
    BSP_Timer_Init();
    GIMBAL_TaskInit();
    COMMUNICATION_ServiceInit();
}

/**
 * @brief 调度云台应用的非阻塞任务
 */
void APP_MSPB_Process(void)
{
    COMMUNICATION_ServiceProcess();
    GIMBAL_TaskProcess();
}
