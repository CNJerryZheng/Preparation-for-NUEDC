/**
 * @file        communication_service.h
 * @author      JerryZheng
 * @brief       云台通信业务服务接口
 * @date        2026-07-21
 */

#pragma once

#include <stdint.h>

/** @brief Live Watch：最近一次收到的底盘轨迹进度，范围0~10000。 */
extern volatile uint16_t g_vehicle_progress;
/** @brief Live Watch：通过校验的底盘进度帧累计数量。 */
extern volatile uint32_t g_progress_frame_count;
/** @brief Live Watch：通过校验的树莓派视觉帧累计数量。 */
extern volatile uint32_t g_vision_frame_count;
/** @brief Live Watch：串口协议校验或格式错误累计数量。 */
extern volatile uint32_t g_protocol_error_count;

/** @brief 初始化云台通信业务状态 */
void COMMUNICATION_ServiceInit(void);
/** @brief 处理已接收的云台通信业务数据 */
void COMMUNICATION_ServiceProcess(void);
