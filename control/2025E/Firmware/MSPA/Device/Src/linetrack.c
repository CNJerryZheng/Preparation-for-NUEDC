/**
 * @file        linetrack.c
 * @author      JerryZheng
 * @brief       八路循迹模块
 * @date        2026-07-12
 */

#include "linetrack.h"
#include "ti_msp_dl_config.h"

/* <----------------数据变量----------------> */
#ifdef LINETRACK_USE_IO
/** @brief 最后一次检测到黑线时的方向，-1 为左，1 为右。 */
static int8_t s_last_direction = 0;
/** @brief 最后一次检测到黑线时的位置。 */
static float s_last_position = 0.0f;
/** @brief X1 至 X8 的位置权重。 */
static const float weight[8] = {
    -3.5f,
    -2.5f,
    -1.5f,
    -0.5f,
    0.5f,
    1.5f,
    2.5f,
    3.5f
};

/**
 * @brief 读取一个循迹输入引脚的逻辑电平
 * @param port GPIO 端口寄存器
 * @param pin GPIO 引脚掩码
 * @return uint8_t 当前黑白状态
 */
static uint8_t LINE_ReadPin(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) != 0U) ? LINETRACK_WHITE : LINETRACK_BLACK;
}
#endif

#ifdef LINETRACK_USE_UART

#endif

/* <------------------函数------------------> */
#ifdef LINETRACK_USE_IO

/**
 * @brief 读取八路循迹模块的原始位图
 * @return uint8_t bit0 至 bit7 分别对应 X1 至 X8
 */
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

/**
 * @brief 从原始位图中取得指定通道的状态
 * @param raw 八路循迹原始位图
 * @param channel 通道编号，0 至 7 分别对应 X1 至 X8
 * @return uint8_t 指定通道的黑白状态
 */
uint8_t LINE_ReadChannel(uint8_t raw, uint8_t channel)
{
    if (channel > 7U)
    {
        return LINETRACK_WHITE;
    }
    return (uint8_t)((raw >> channel) & 0x01U);
}

/**
 * @brief 读取八路循迹传感器并计算黑线位置与状态
 * @return LINE_Result_t 当前循迹计算结果
 */
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
        if (s_last_direction < 0)
        {
            result.state = State_Lose_Left;
            return result;
        }
        if (s_last_direction > 0)
        {
            result.state = State_Lose_Right;
            return result;
        }
        result.state = State_Lose_Unknown;
        return result;
    }

    /* 八路全黑，可能为十字路口或黑色区域。 */
    if (raw == 0x00U)
    {
        result.black_count = 8U;
        result.state = State_All_Black;
        return result;
    }

    float position_sum = 0.0f;
    uint8_t black_count = 0U;
    uint8_t first_black = 8U;
    uint8_t last_black = 0U; // 黑总数，起始黑，最后黑

    /* 统计所有检测到黑线的传感器，并计算位置权重之和 */
    for (uint8_t i = 0U; i < 8U; i++)
    {
        if (LINE_ReadChannel(raw, i) == LINETRACK_BLACK)
        {
            if (black_count == 0U)
            {
                first_black = i;
            }

            last_black = i;
            black_count++;
            position_sum += weight[i];
        }
    }
    result.black_count = black_count;
    if ((black_count == 0U) || (black_count == 8U)) // 再次确认，保护
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
