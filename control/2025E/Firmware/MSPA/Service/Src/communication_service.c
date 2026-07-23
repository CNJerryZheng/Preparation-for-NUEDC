/**
 * @file        communication_service.c
 * @author      JerryZheng
 * @brief       底盘通信业务服务实现
 * @date        2026-07-21
 */

#include "communication_service.h"
#include "communication_service_config.h"
#include "trajectory_progress_service.h"

#if COMMUNICATION_VOFA_DEBUG_ENABLE
#include "vofa_service.h"
#endif

/**
 * @brief 初始化底盘通信业务状态
 */
void COMMUNICATION_ServiceInit(void)
{
    TRAJECTORY_ProgressServiceInit();
#if COMMUNICATION_VOFA_DEBUG_ENABLE
    VOFA_ServiceInit();
#endif
}

/**
 * @brief 处理已接收的底盘通信业务数据
 */
void COMMUNICATION_ServiceProcess(void)
{
    TRAJECTORY_ProgressServiceProcess();
#if COMMUNICATION_VOFA_DEBUG_ENABLE
    VOFA_ServiceProcess();
#endif
}
