/**
 * @file        app_mspa.h
 * @author      JerryZheng
 * @brief       MSPA 底盘应用调度层接口
 * @date        2026-07-21
 */

#pragma once

/**
 * @brief 初始化 MSPA 底盘应用服务
 */
void APP_MSPA_Init(void);

/**
 * @brief 执行一次 MSPA 非阻塞应用调度
 */
void APP_MSPA_Process(void);
