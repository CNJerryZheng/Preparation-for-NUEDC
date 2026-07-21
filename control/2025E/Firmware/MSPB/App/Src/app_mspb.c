/**
 * @file        app_mspb.c
 * @author      JerryZheng
 * @brief       MSPB application entry.
 * @date        2026-07-20
 */

#include "app_mspb.h"
#include "bsp_uart.h"
#include "gimbal_task.h"
#include "communication_service.h"

void APP_MSPB_Init(void)
{
    BSP_UART_Init();
    GIMBAL_TaskInit();
    COMMUNICATION_ServiceInit();
}

void APP_MSPB_Process(void)
{
    COMMUNICATION_ServiceProcess();
    GIMBAL_TaskProcess();
}
