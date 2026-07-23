/**
 * @file        vofa_service.h
 * @author      JerryZheng
 * @brief       VOFA+ 轮速 PID 调参服务接口
 * @date        2026-07-22
 */

#pragma once

/**
 * @brief 初始化 VOFA+ 调参服务
 */
void VOFA_ServiceInit(void);

/**
 * @brief 处理 VOFA+ 命令并按 50Hz 上传 JustFloat 波形
 */
void VOFA_ServiceProcess(void);
