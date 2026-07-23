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
    // 正式模式始终启用轨迹进度服务，VOFA+仅由编译开关选择。
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
    // 持续累计并上报底盘进度，调试模式下同时处理在线调参。
    TRAJECTORY_ProgressServiceProcess();
#if COMMUNICATION_VOFA_DEBUG_ENABLE
    VOFA_ServiceProcess();
#endif
}
