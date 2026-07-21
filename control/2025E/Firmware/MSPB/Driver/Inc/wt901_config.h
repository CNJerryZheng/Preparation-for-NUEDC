/**
 * @file        wt901_config.h
 * @author      Misybon
 *             JerryZheng
 * @brief       WT901 stream-parser configuration.
 * @date        2026-07-20
 */

#pragma once

/* Keep the F407 parser's buffer sizing and 11-byte protocol framing. */
#define WT901_CIR_SIZE      254U
#define WT901_BUF_SIZE      128U
#define WT901_FRAME_SIZE    11U
#define WT901_FRAME_HEADER  0x55U
