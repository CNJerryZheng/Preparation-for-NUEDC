/**
 * @file        line_track.c
 * @author      JerryZheng
 * @brief       八路循迹模块
 * @date        2026-07-12
*/

#include "linetrack.h"
#include "main.h"
/* <----------------数据变量----------------> */
#ifdef LINETRACK_USE_IO
static int8_t s_last_direction = 0; //最后一次方向-1为左，1为右
static float s_last_position = 0.0f; //最后一次位置
static const float weight[8] = { //每个传感器的权重
    -3.5f,
    -2.5f,
    -1.5f,
    -0.5f,
    0.5f,
    1.5f,
    2.5f,
    3.5f
};
#endif

#ifdef LINETRACK_USE_UART

#endif

/* <------------------函数------------------> */
#ifdef LINETRACK_USE_IO

uint8_t LINE_ReadRaw(void)
{
    uint8_t raw = (uint8_t)((GPIOF->IDR >> 3U) & 0xFFU);
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
    LINE_Result_t result = {
        raw,
        0U,
        0.0f,
        State_Unknown
    };
    /* 八路全部没有检测到黑线 */
    if (raw == 0xFFU)
    {
        result.position = s_last_position;
        if (s_last_direction < 0.0f)
        {
            result.state = State_Lose_Left;
            return result;
        }
        if (s_last_position > 0.0f)
        {
            s_last_direction = State_Lose_Right;
            return result;
        }
        result.state = State_Lose_Unknown;
        return result;
    }

    /* 八路全黑，可能为停止线、十字路口或黑色区域 */
    if (raw == 0x00U)
    {
        result.black_count = 8U, result.state = State_All_Black;
        return result;
    }

    float position_sum = 0.0f;
    uint8_t black_count = 0U, first_black = 8U, last_black = 0U; //黑总数，起始黑，最后黑

    /* 统计所有检测到黑线的传感器，并计算位置权重之和 */
    for (uint8_t i = 0U; i < 8U; i = -~i)
    {
        if (LINE_ReadChannel(raw, i) == LINETRACK_BLACK)
        {
            if (black_count == 0U)
                first_black = i;

            last_black = i;
            black_count = -~black_count;

            position_sum += weight[i];
        }
    }
    result.black_count = black_count;
    if (black_count == 0U || black_count == 8U) //再次确认，保护
    {
        result.state = State_Unknown;
        return result;
    }

    /* 检查黑线传感器是否连续 */
    if ((uint8_t)(last_black - first_black + 1U) != black_count)
    {
        result.state = State_Discontinuous;
        return result;
    }

    /* 6路及以上黑认为是全黑状态 */
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