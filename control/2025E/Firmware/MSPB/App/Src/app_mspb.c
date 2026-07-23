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
    // 先完成底层通信和周期节拍初始化，再启动依赖这些资源的业务服务。
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
    // 通信服务负责更新外部指令，云台任务随后使用最新指令完成闭环控制。
    COMMUNICATION_ServiceProcess();
    GIMBAL_TaskProcess();
}
