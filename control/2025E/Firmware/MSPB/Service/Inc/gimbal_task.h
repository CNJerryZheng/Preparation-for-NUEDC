/**
 * @file        gimbal_task.h
 * @author      JerryZheng
 * @brief       云台周期业务服务接口
 * @date        2026-07-21
 */

#pragma once

/** @brief 初始化云台周期业务状态 */
void GIMBAL_TaskInit(void);
/** @brief 执行一次非阻塞云台周期业务 */
void GIMBAL_TaskProcess(void);
