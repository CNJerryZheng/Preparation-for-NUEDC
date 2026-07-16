/**
 * @file        line_track.h
 * @author      JerryZheng
 * @brief       八路循迹模块
 * @date        2026-07-12
*/
#pragma once

#include <stdint.h>
#include "linetrack_config.h"

#ifdef LINETRACK_USE_IO
/* <---------------IO的数据结构---------------> */
/**
 * @brief 特殊状态
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
 * @brief 返回处理后的信息
 */
typedef struct
{
    uint8_t raw;
    uint8_t black_count;

    float position;

    LINE_State_t state;

} LINE_Result_t;

/* <---------------IO的函数部分---------------> */
/**
 * @brief 读取IO数据
 *
 * @retval uint8_t  读取的生数据
 */
uint8_t LINE_ReadRaw(void);

/**
 * @brief 读取一个引脚的数据
 *
 * @param raw 生数据
 * @param channel 引脚 值为 0U->7U 对应 X1->X8
 */
uint8_t LINE_ReadChannel(uint8_t raw, uint8_t channel);
LINE_Result_t LINE_Process(void);
#endif

#ifdef LINETRACK_USE_UART
/* <---------------串口的数据结构---------------> */

/* <---------------串口的函数部分---------------> */

#endif
