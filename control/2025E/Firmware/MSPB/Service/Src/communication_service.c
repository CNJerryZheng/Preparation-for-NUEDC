/**
 * @file        communication_service.c
 * @author      JerryZheng
 * @brief       云台通信业务服务实现
 * @date        2026-07-21
 */

#include "communication_service.h"
#include "bsp_uart.h"
#include "gimbal_axis.h"

#define COMMUNICATION_FRAME_HEADER_1 (0xAAU)
#define COMMUNICATION_FRAME_HEADER_2 (0x55U)
#define COMMUNICATION_COMMAND_PROGRESS (0x01U)
#define COMMUNICATION_COMMAND_VISION (0x02U)
#define COMMUNICATION_MAX_PAYLOAD_LENGTH (16U)

/**
 * @brief 串口协议解析状态
 */
typedef enum
{
    COMMUNICATION_PARSE_HEADER_1 = 0,
    COMMUNICATION_PARSE_HEADER_2,
    COMMUNICATION_PARSE_COMMAND,
    COMMUNICATION_PARSE_LENGTH,
    COMMUNICATION_PARSE_PAYLOAD,
    COMMUNICATION_PARSE_CHECKSUM
} COMMUNICATION_ParseState_t;

/**
 * @brief 单路串口协议解析器
 */
typedef struct
{
    COMMUNICATION_ParseState_t state; /**< 当前解析状态。 */
    uint8_t command;                  /**< 当前帧命令字。 */
    uint8_t length;                   /**< 当前帧载荷长度。 */
    uint8_t payload_index;            /**< 当前已接收载荷字节数。 */
    uint8_t checksum;                 /**< 当前异或校验累计值。 */
    uint8_t payload[COMMUNICATION_MAX_PAYLOAD_LENGTH]; /**< 帧载荷。 */
} COMMUNICATION_Parser_t;

/** @brief UART2底盘进度协议解析器。 */
static COMMUNICATION_Parser_t s_mspa_parser;
/** @brief UART3树莓派视觉协议解析器。 */
static COMMUNICATION_Parser_t s_rpi_parser;

volatile uint16_t g_vehicle_progress = 0U;
volatile uint32_t g_progress_frame_count = 0U;
volatile uint32_t g_vision_frame_count = 0U;
volatile uint32_t g_protocol_error_count = 0U;

/** @brief Live Watch：最近一次视觉目标有效标志。 */
volatile uint8_t g_vision_ok = 0U;
/** @brief Live Watch：最近一次激光目标图像X坐标。 */
volatile int16_t g_vision_laser_x = 0;
/** @brief Live Watch：最近一次激光目标图像Y坐标。 */
volatile int16_t g_vision_laser_y = 0;
/** @brief Live Watch：最近一次靶心X偏差。 */
volatile int16_t g_vision_center_x = 0;
/** @brief Live Watch：最近一次靶心Y偏差。 */
volatile int16_t g_vision_center_y = 0;

/**
 * @brief 将协议解析器恢复到等待包头状态
 * @param parser 协议解析器地址
 */
static void COMMUNICATION_ResetParser(COMMUNICATION_Parser_t *parser)
{
    parser->state = COMMUNICATION_PARSE_HEADER_1;
    parser->command = 0U;
    parser->length = 0U;
    parser->payload_index = 0U;
    parser->checksum = 0U;
}

/**
 * @brief 向协议解析器输入一个字节
 * @param parser 协议解析器地址
 * @param data 新收到的字节
 * @return true 已收到一帧且校验正确
 * @return false 当前尚未形成完整有效帧
 */
static bool COMMUNICATION_ParseByte(
    COMMUNICATION_Parser_t *parser, uint8_t data)
{
    switch (parser->state)
    {
    case COMMUNICATION_PARSE_HEADER_1:
        if (data == COMMUNICATION_FRAME_HEADER_1)
        {
            parser->state = COMMUNICATION_PARSE_HEADER_2;
        }
        break;

    case COMMUNICATION_PARSE_HEADER_2:
        if (data == COMMUNICATION_FRAME_HEADER_2)
        {
            parser->state = COMMUNICATION_PARSE_COMMAND;
        }
        else
        {
            parser->state = (data == COMMUNICATION_FRAME_HEADER_1) ?
                COMMUNICATION_PARSE_HEADER_2 :
                COMMUNICATION_PARSE_HEADER_1;
        }
        break;

    case COMMUNICATION_PARSE_COMMAND:
        parser->command = data;
        parser->checksum = data;
        parser->state = COMMUNICATION_PARSE_LENGTH;
        break;

    case COMMUNICATION_PARSE_LENGTH:
        parser->length = data;
        parser->checksum ^= data;
        parser->payload_index = 0U;
        if (parser->length > COMMUNICATION_MAX_PAYLOAD_LENGTH)
        {
            ++g_protocol_error_count;
            COMMUNICATION_ResetParser(parser);
        }
        else
        {
            parser->state = (parser->length == 0U) ?
                COMMUNICATION_PARSE_CHECKSUM :
                COMMUNICATION_PARSE_PAYLOAD;
        }
        break;

    case COMMUNICATION_PARSE_PAYLOAD:
        parser->payload[parser->payload_index++] = data;
        parser->checksum ^= data;
        if (parser->payload_index >= parser->length)
        {
            parser->state = COMMUNICATION_PARSE_CHECKSUM;
        }
        break;

    case COMMUNICATION_PARSE_CHECKSUM:
    {
        const bool valid = data == parser->checksum;

        if (!valid)
        {
            ++g_protocol_error_count;
        }
        parser->state = COMMUNICATION_PARSE_HEADER_1;
        return valid;
    }

    default:
        COMMUNICATION_ResetParser(parser);
        break;
    }
    return false;
}

/**
 * @brief 从小端序载荷读取一个有符号16位整数
 * @param data 两字节数据地址
 * @return int16_t 解码后的有符号整数
 */
static int16_t COMMUNICATION_ReadInt16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] |
        ((uint16_t)data[1] << 8U));
}

/**
 * @brief 将有效底盘进度帧原样转发给树莓派
 * @param parser 已完成解析的底盘协议帧
 */
static void COMMUNICATION_HandleProgress(
    const COMMUNICATION_Parser_t *parser)
{
    uint8_t frame[7];
    uint16_t progress;

    if ((parser->command != COMMUNICATION_COMMAND_PROGRESS) ||
        (parser->length != 2U))
    {
        ++g_protocol_error_count;
        return;
    }

    progress = (uint16_t)parser->payload[0] |
        ((uint16_t)parser->payload[1] << 8U);
    if (progress > 10000U)
    {
        progress = 10000U;
    }

    g_vehicle_progress = progress;
    ++g_progress_frame_count;
    frame[0] = COMMUNICATION_FRAME_HEADER_1;
    frame[1] = COMMUNICATION_FRAME_HEADER_2;
    frame[2] = COMMUNICATION_COMMAND_PROGRESS;
    frame[3] = 2U;
    frame[4] = (uint8_t)(progress & 0xFFU);
    frame[5] = (uint8_t)(progress >> 8U);
    frame[6] = frame[2] ^ frame[3] ^ frame[4] ^ frame[5];
    BSP_UART_RpiWrite(frame, (uint16_t)sizeof(frame));
}

/**
 * @brief 将有效树莓派视觉帧提交给云台闭环
 * @param parser 已完成解析的视觉协议帧
 */
static void COMMUNICATION_HandleVision(
    const COMMUNICATION_Parser_t *parser)
{
    if ((parser->command != COMMUNICATION_COMMAND_VISION) ||
        (parser->length != 9U))
    {
        ++g_protocol_error_count;
        return;
    }

    g_vision_ok = parser->payload[0];
    g_vision_laser_x = COMMUNICATION_ReadInt16(&parser->payload[1]);
    g_vision_laser_y = COMMUNICATION_ReadInt16(&parser->payload[3]);
    g_vision_center_x = COMMUNICATION_ReadInt16(&parser->payload[5]);
    g_vision_center_y = COMMUNICATION_ReadInt16(&parser->payload[7]);
    ++g_vision_frame_count;
    GIMBAL_AxisSetVisionTarget(g_vision_ok != 0U,
        g_vision_laser_x, g_vision_laser_y);
}

/**
 * @brief 初始化云台通信业务状态
 */
void COMMUNICATION_ServiceInit(void)
{
    COMMUNICATION_ResetParser(&s_mspa_parser);
    COMMUNICATION_ResetParser(&s_rpi_parser);
    g_vehicle_progress = 0U;
    g_progress_frame_count = 0U;
    g_vision_frame_count = 0U;
    g_protocol_error_count = 0U;
    g_vision_ok = 0U;
    g_vision_laser_x = 0;
    g_vision_laser_y = 0;
    g_vision_center_x = 0;
    g_vision_center_y = 0;
}

/**
 * @brief 处理已接收的云台通信业务数据
 */
void COMMUNICATION_ServiceProcess(void)
{
    uint8_t data;

    while (BSP_UART_MspaReadByte(&data))
    {
        if (COMMUNICATION_ParseByte(&s_mspa_parser, data))
        {
            COMMUNICATION_HandleProgress(&s_mspa_parser);
        }
    }

    while (BSP_UART_RpiReadByte(&data))
    {
        if (COMMUNICATION_ParseByte(&s_rpi_parser, data))
        {
            COMMUNICATION_HandleVision(&s_rpi_parser);
        }
    }
}
