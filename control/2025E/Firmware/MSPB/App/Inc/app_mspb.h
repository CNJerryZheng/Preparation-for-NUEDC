/**
 * @file        app_mspb.h
 * @author      JerryZheng
 * @brief       MSPB 云台应用调度层接口
 * @date        2026-07-21
 */

#pragma once

/** @brief 初始化 MSPB 云台应用服务 */
void APP_MSPB_Init(void);
/** @brief 执行一次 MSPB 非阻塞应用调度 */
void APP_MSPB_Process(void);
