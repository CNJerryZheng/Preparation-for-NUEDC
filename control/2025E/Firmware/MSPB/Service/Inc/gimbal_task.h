/**
 * @file        gimbal_task.h
 * @author      JerryZheng
 * @brief       Gimbal periodic service task.
 * @date        2026-07-20
 */

#pragma once

/** @brief Initialize gimbal task state. */
void GIMBAL_TaskInit(void);
/** @brief Process one non-blocking gimbal task cycle. */
void GIMBAL_TaskProcess(void);
