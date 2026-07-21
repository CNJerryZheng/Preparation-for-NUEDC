/**
 * @file        app_mspb.h
 * @author      JerryZheng
 * @brief       MSPB application entry.
 * @date        2026-07-20
 */

#pragma once

/** @brief Initialize MSPB application services. */
void APP_MSPB_Init(void);
/** @brief Run one non-blocking MSPB application cycle. */
void APP_MSPB_Process(void);
