/**
 * @file        app_mspa.c
 * @author      JerryZheng
 * @brief       MSPA application entry.
 * @date        2026-07-20
 */

#include "app_mspa.h"
#include "bsp_uart.h"
#include "chassis_task.h"
#include "communication_service.h"

void APP_MSPA_Init(void)
{
    BSP_UART_Init();
    CHASSIS_TaskInit();
    COMMUNICATION_ServiceInit();
}

void APP_MSPA_Process(void)
{
    COMMUNICATION_ServiceProcess();
    CHASSIS_TaskProcess();
}
