/**
 * @file        communication_service.h
 * @author      JerryZheng
 * @brief       底盘通信业务服务接口
 * @date        2026-07-21
 */

#pragma once

/** @brief 初始化底盘通信业务状态 */
void COMMUNICATION_ServiceInit(void);
/** @brief 处理已接收的底盘通信业务数据 */
void COMMUNICATION_ServiceProcess(void);
