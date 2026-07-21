/**
 * @file        chassis_task.h
 * @author      JerryZheng
 * @brief       底盘周期业务服务接口
 * @date        2026-07-21
 */

#pragma once

/** @brief 初始化底盘周期业务状态 */
void CHASSIS_TaskInit(void);
/** @brief 执行一次非阻塞底盘周期业务 */
void CHASSIS_TaskProcess(void);
