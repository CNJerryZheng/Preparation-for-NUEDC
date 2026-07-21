/**
 * @file        linetrack.h
 * @author      JerryZheng
 * @brief       Eight-channel infrared line-tracking module.
 * @date        2026-07-20
 */

#pragma once

#include <stdint.h>
#include "linetrack_config.h"

#ifdef LINETRACK_USE_IO
/**
 * @brief Line state decoded from the eight sensor inputs.
 */
typedef enum
{
    State_OK = 0,
    State_Lose_Left,
    State_Lose_Right,
    State_Lose_Unknown,
    State_All_Black,
    State_Unknown,
    State_Discontinuous
} LINE_State_t;

/**
 * @brief Processed line-tracking data.
 */
typedef struct
{
    uint8_t raw;
    uint8_t black_count;
    float position;
    LINE_State_t state;
} LINE_Result_t;

/**
 * @brief Read all sensor inputs, bit 0 through bit 7 map to LINE_1 through LINE_8.
 * @return Active-low raw sensor bitmap.
 */
uint8_t LINE_ReadRaw(void);

/**
 * @brief Read one channel from a raw sensor bitmap.
 * @param raw Raw sensor bitmap.
 * @param channel Channel 0 through 7, corresponding to LINE_1 through LINE_8.
 * @return The electrical level of the requested channel.
 */
uint8_t LINE_ReadChannel(uint8_t raw, uint8_t channel);

/**
 * @brief Decode the current sensor inputs into position and state.
 * @return Decoded line result.
 */
LINE_Result_t LINE_Process(void);
#endif
