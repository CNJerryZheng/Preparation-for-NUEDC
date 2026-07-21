/**
 * @file        app_mspa.h
 * @author      JerryZheng
 * @brief       MSPA application entry.
 * @date        2026-07-20
 */

#pragma once

/**
 * @brief Initialize MSPA application services.
 */
void APP_MSPA_Init(void);

/**
 * @brief Run one non-blocking MSPA application cycle.
 */
void APP_MSPA_Process(void);
