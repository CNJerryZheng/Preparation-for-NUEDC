/**
 * @file        app_mspa.c
 * @author      JerryZheng
 * @brief       MSPA 底盘应用调度层
 * @date        2026-07-21
 */

#include "app_mspa.h"
#include "bsp_timer.h"
#include "bsp_uart.h"
#include "chassis_task.h"
#include "communication_service.h"

/**
 * @brief 初始化底盘应用所需的 BSP 与服务层
 */
void APP_MSPA_Init(void)
{
    // 先初始化底层串口和控制节拍，再启动依赖硬件资源的底盘业务。
    BSP_UART_Init();
    // 调用 BSP_Timer_Init，初始化对应模块或运行状态。
    BSP_Timer_Init();
    // 调用 CHASSIS_TaskInit，初始化对应模块或运行状态。
    CHASSIS_TaskInit();
    // 调用 COMMUNICATION_ServiceInit，初始化对应模块或运行状态。
    COMMUNICATION_ServiceInit();
}

/**
 * @brief 调度底盘应用的非阻塞任务
 */
void APP_MSPA_Process(void)
{
    // 底盘控制优先消费周期节拍，通信服务随后处理进度上报和可选调试。
    CHASSIS_TaskProcess();
    // 调用 COMMUNICATION_ServiceProcess，更新并处理对应业务数据。
    COMMUNICATION_ServiceProcess();
}
