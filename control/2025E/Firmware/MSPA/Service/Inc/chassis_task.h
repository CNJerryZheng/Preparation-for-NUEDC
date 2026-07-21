/**
 * @file        chassis_task.h
 * @author      JerryZheng
 * @brief       Chassis periodic service task.
 * @date        2026-07-20
 */

#pragma once

/** @brief Initialize chassis task state. */
void CHASSIS_TaskInit(void);
/** @brief Process one non-blocking chassis task cycle. */
void CHASSIS_TaskProcess(void);
