/**
 * @file        communication_service.h
 * @author      JerryZheng
 * @brief       Chassis communication service interface.
 * @date        2026-07-20
 */

#pragma once

/** @brief Initialize chassis communication state. */
void COMMUNICATION_ServiceInit(void);
/** @brief Process queued chassis communication data. */
void COMMUNICATION_ServiceProcess(void);
