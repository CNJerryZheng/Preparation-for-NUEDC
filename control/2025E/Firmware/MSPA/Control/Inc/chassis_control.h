/**
 * @file        chassis_control.h
 * @author      JerryZheng
 * @brief       Chassis control-layer interface.
 * @date        2026-07-20
 */

#pragma once

#include "linetrack.h"

/**
 * @brief Update the chassis control target from a line result.
 * @param line Latest line-tracking result.
 */
void CHASSIS_ControlUpdate(const LINE_Result_t *line);
