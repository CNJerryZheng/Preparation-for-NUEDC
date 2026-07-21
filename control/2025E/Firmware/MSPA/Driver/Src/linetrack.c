/**
 * @file        linetrack.c
 * @author      JerryZheng
 * @brief       Eight-channel infrared line-tracking module.
 * @date        2026-07-20
 */

#include "linetrack.h"
#include "ti_msp_dl_config.h"

#ifdef LINETRACK_USE_IO
static int8_t s_last_direction = 0;
static float s_last_position = 0.0f;
static const float s_weight[8] = {
    -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f
};

/**
 * @brief Convert one MSPM0 GPIO input to the original raw-bit convention.
 */
static uint8_t LINE_ReadPin(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) != 0U) ? LINETRACK_WHITE : LINETRACK_BLACK;
}

uint8_t LINE_ReadRaw(void)
{
    uint8_t raw = 0U;

    raw |= (uint8_t)(LINE_ReadPin(GPIO_LINE_SENSOR_LINE_1_PORT, GPIO_LINE_SENSOR_LINE_1_PIN) << 0U);
    raw |= (uint8_t)(LINE_ReadPin(GPIO_LINE_SENSOR_LINE_2_PORT, GPIO_LINE_SENSOR_LINE_2_PIN) << 1U);
    raw |= (uint8_t)(LINE_ReadPin(GPIO_LINE_SENSOR_LINE_3_PORT, GPIO_LINE_SENSOR_LINE_3_PIN) << 2U);
    raw |= (uint8_t)(LINE_ReadPin(GPIO_LINE_SENSOR_LINE_4_PORT, GPIO_LINE_SENSOR_LINE_4_PIN) << 3U);
    raw |= (uint8_t)(LINE_ReadPin(GPIO_LINE_SENSOR_LINE_5_PORT, GPIO_LINE_SENSOR_LINE_5_PIN) << 4U);
    raw |= (uint8_t)(LINE_ReadPin(GPIO_LINE_SENSOR_LINE_6_PORT, GPIO_LINE_SENSOR_LINE_6_PIN) << 5U);
    raw |= (uint8_t)(LINE_ReadPin(GPIO_LINE_SENSOR_LINE_7_PORT, GPIO_LINE_SENSOR_LINE_7_PIN) << 6U);
    raw |= (uint8_t)(LINE_ReadPin(GPIO_LINE_SENSOR_LINE_8_PORT, GPIO_LINE_SENSOR_LINE_8_PIN) << 7U);
    return raw;
}

uint8_t LINE_ReadChannel(uint8_t raw, uint8_t channel)
{
    if (channel > 7U)
    {
        return LINETRACK_WHITE;
    }
    return (uint8_t)((raw >> channel) & 0x01U);
}

LINE_Result_t LINE_Process(void)
{
    uint8_t raw = LINE_ReadRaw();
    LINE_Result_t result = { raw, 0U, 0.0f, State_Unknown };

    if (raw == 0xFFU)
    {
        result.position = s_last_position;
        if (s_last_direction < 0)
        {
            result.state = State_Lose_Left;
        }
        else if (s_last_direction > 0)
        {
            result.state = State_Lose_Right;
        }
        else
        {
            result.state = State_Lose_Unknown;
        }
        return result;
    }

    if (raw == 0x00U)
    {
        result.black_count = 8U;
        result.state = State_All_Black;
        return result;
    }

    float position_sum = 0.0f;
    uint8_t black_count = 0U;
    uint8_t first_black = 8U;
    uint8_t last_black = 0U;

    for (uint8_t index = 0U; index < 8U; index++)
    {
        if (LINE_ReadChannel(raw, index) == LINETRACK_BLACK)
        {
            if (black_count == 0U)
            {
                first_black = index;
            }
            last_black = index;
            black_count++;
            position_sum += s_weight[index];
        }
    }

    result.black_count = black_count;
    if ((black_count == 0U) || (black_count == 8U))
    {
        result.state = State_Unknown;
        return result;
    }
    if ((uint8_t)(last_black - first_black + 1U) != black_count)
    {
        result.state = State_Discontinuous;
        return result;
    }
    if (black_count >= LINETRACK_MAX_BLACK)
    {
        result.state = State_All_Black;
        return result;
    }

    result.state = State_OK;
    result.position = position_sum / (float)black_count;
    s_last_position = result.position;
    if (result.position < 0.0f)
    {
        s_last_direction = -1;
    }
    else if (result.position > 0.0f)
    {
        s_last_direction = 1;
    }
    return result;
}
#endif
