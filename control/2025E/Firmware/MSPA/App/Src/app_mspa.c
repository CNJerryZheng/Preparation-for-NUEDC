/**
 * @file        app_mspa.c
 * @author      JerryZheng
 * @brief       MSPA 底盘应用调度层
 * @date        2026-07-21
 */

#include "app_mspa.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include "chassis_task.h"
#include "communication_service.h"

/**
 * @brief 初始化底盘应用所需的 BSP 与服务层
 */
void APP_MSPA_Init(void)
{
    BSP_UART_Init();
    BSP_Timer_Init();
    CHASSIS_TaskInit();
    COMMUNICATION_ServiceInit();
}

/**
 * @brief 调度底盘应用的非阻塞任务
 */
void APP_MSPA_Process(void)
{
    COMMUNICATION_ServiceProcess();
    CHASSIS_TaskProcess();
}
