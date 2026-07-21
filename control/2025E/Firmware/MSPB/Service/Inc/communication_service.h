/**
 * @file        communication_service.h
 * @author      JerryZheng
 * @brief       Gimbal communication service interface.
 * @date        2026-07-20
 */

#pragma once

/** @brief Initialize gimbal communication state. */
void COMMUNICATION_ServiceInit(void);
/** @brief Process queued gimbal communication data. */
void COMMUNICATION_ServiceProcess(void);
