/**
 * @file        linetrack_config.h
 * @author      JerryZheng
 * @brief       Eight-channel infrared line-tracking configuration.
 * @date        2026-07-20
 */

#pragma once

/* The original F407 module uses active-low sensor outputs. */
#define LINETRACK_USE_IO
#define LINETRACK_BLACK      0U
#define LINETRACK_WHITE      1U
#define LINETRACK_MAX_BLACK  6U
