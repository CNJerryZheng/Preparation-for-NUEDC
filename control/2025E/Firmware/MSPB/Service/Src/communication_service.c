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
    // 更新 COMMUNICATION_PARSE_HEADER_1 对应的本步骤的运行数据。
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
    // 定义本步骤需要的局部数据并完成初始化。
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

// 更新 g_vehicle_progress 对应的进度状态。
volatile uint16_t g_vehicle_progress = 0U;
// 更新 g_progress_frame_count 对应的协议帧字段。
volatile uint32_t g_progress_frame_count = 0U;
// 更新 g_vision_frame_count 对应的协议帧字段。
volatile uint32_t g_vision_frame_count = 0U;
// 更新 g_protocol_error_count 对应的累计计数。
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
    // 丢弃未完成帧并重新等待第一个包头字节。
    parser->state = COMMUNICATION_PARSE_HEADER_1;
    // 更新 parser->command，保存当前语句的计算结果。
    parser->command = 0U;
    // 更新 parser->length，保存当前语句的计算结果。
    parser->length = 0U;
    // 更新载荷写入位置，指向下一待接收字节。
    parser->payload_index = 0U;
    // 更新 parser->checksum 的协议异或校验值。
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
    // 状态机逐字节解析，避免在串口中断中执行完整协议业务。
    switch (parser->state)
    {
    // 处理当前枚举值对应的业务状态。
    case COMMUNICATION_PARSE_HEADER_1:
        // 判断 data == COMMUNICATION_FRAME_HEADER_1 是否成立，并选择对应处理分支。
        if (data == COMMUNICATION_FRAME_HEADER_1)
        {
            // 更新 parser->state 的状态机状态。
            parser->state = COMMUNICATION_PARSE_HEADER_2;
        }
        // 执行 break，完成当前业务步骤。
        break;

    // 处理当前枚举值对应的业务状态。
    case COMMUNICATION_PARSE_HEADER_2:
        // 判断 data == COMMUNICATION_FRAME_HEADER_2 是否成立，并选择对应处理分支。
        if (data == COMMUNICATION_FRAME_HEADER_2)
        {
            // 更新 parser->state 的状态机状态。
            parser->state = COMMUNICATION_PARSE_COMMAND;
        }
        // 前述条件均不成立时执行备用处理。
        else
        {
            // 当前字节仍为首包头时直接保留同步，否则完全重新找包头。
            parser->state = (data == COMMUNICATION_FRAME_HEADER_1) ?
                COMMUNICATION_PARSE_HEADER_2 :
                // 执行 COMMUNICATION_PARSE_HEADER_1，完成当前业务步骤。
                COMMUNICATION_PARSE_HEADER_1;
        }
        // 执行 break，完成当前业务步骤。
        break;

    // 处理当前枚举值对应的业务状态。
    case COMMUNICATION_PARSE_COMMAND:
        // 更新 parser->command，保存当前语句的计算结果。
        parser->command = data;
        // 更新 parser->checksum 的协议异或校验值。
        parser->checksum = data;
        // 更新 parser->state 的状态机状态。
        parser->state = COMMUNICATION_PARSE_LENGTH;
        // 执行 break，完成当前业务步骤。
        break;

    // 处理当前枚举值对应的业务状态。
    case COMMUNICATION_PARSE_LENGTH:
        // 更新 parser->length，保存当前语句的计算结果。
        parser->length = data;
        // 更新 ^，保存当前语句的计算结果。
        parser->checksum ^= data;
        // 更新载荷写入位置，指向下一待接收字节。
        parser->payload_index = 0U;
        // 判断 parser->length > COMMUNICATION_MAX_PAYLOAD_LENGTH 是否成立，并选择对应处理分支。
        if (parser->length > COMMUNICATION_MAX_PAYLOAD_LENGTH)
        {
            // 非法长度可能造成数组越界，因此立即丢弃整帧。
            ++g_protocol_error_count;
            // 调用 COMMUNICATION_ResetParser，复位对应模块的历史状态。
            COMMUNICATION_ResetParser(parser);
        }
        // 前述条件均不成立时执行备用处理。
        else
        {
            parser->state = (parser->length == 0U) ?
                COMMUNICATION_PARSE_CHECKSUM :
                // 执行 COMMUNICATION_PARSE_PAYLOAD，完成当前业务步骤。
                COMMUNICATION_PARSE_PAYLOAD;
        }
        // 执行 break，完成当前业务步骤。
        break;

    // 处理当前枚举值对应的业务状态。
    case COMMUNICATION_PARSE_PAYLOAD:
        // 更新对应的累计计数或遍历位置。
        parser->payload[parser->payload_index++] = data;
        // 更新 ^，保存当前语句的计算结果。
        parser->checksum ^= data;
        // 判断 parser->payload_index >= parser->length 是否成立，并选择对应处理分支。
        if (parser->payload_index >= parser->length)
        {
            // 更新 parser->state 的状态机状态。
            parser->state = COMMUNICATION_PARSE_CHECKSUM;
        }
        // 执行 break，完成当前业务步骤。
        break;

    // 处理当前枚举值对应的业务状态。
    case COMMUNICATION_PARSE_CHECKSUM:
    {
        // 更新 valid 对应的有效或使能状态。
        const bool valid = data == parser->checksum;

        // 无论校验是否通过，当前帧结束后都回到包头搜索状态。
        if (!valid)
        {
            // 更新对应的累计计数或遍历位置。
            ++g_protocol_error_count;
        }
        // 更新 parser->state 的状态机状态。
        parser->state = COMMUNICATION_PARSE_HEADER_1;
        // 返回本次计算或查询得到的结果。
        return valid;
    }

    // 处理未明确匹配的默认状态。
    default:
        // 调用 COMMUNICATION_ResetParser，复位对应模块的历史状态。
        COMMUNICATION_ResetParser(parser);
        // 执行 break，完成当前业务步骤。
        break;
    }
    // 返回失败状态，表示当前条件尚未满足。
    return false;
}

/**
 * @brief 从小端序载荷读取一个有符号16位整数
 * @param data 两字节数据地址
 * @return int16_t 解码后的有符号整数
 */
static int16_t COMMUNICATION_ReadInt16(const uint8_t *data)
{
    // 返回本次计算或查询得到的结果。
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
    // 定义本步骤需要的局部数据并完成初始化。
    uint8_t frame[7];
    // 定义本步骤需要的局部数据并完成初始化。
    uint16_t progress;

    // 只接受固定命令字和两字节进度载荷。
    if ((parser->command != COMMUNICATION_COMMAND_PROGRESS) ||
        (parser->length != 2U))
    {
        // 更新对应的累计计数或遍历位置。
        ++g_protocol_error_count;
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 更新 progress 对应的进度状态。
    progress = (uint16_t)parser->payload[0] |
        ((uint16_t)parser->payload[1] << 8U);
    // 协议满量程为10000，异常上位机数据按100%处理。
    if (progress > 10000U)
    {
        // 更新 progress 对应的进度状态。
        progress = 10000U;
    }

    // 保存本地进度后重组标准帧并转发给树莓派。
    g_vehicle_progress = progress;
    // 更新对应的累计计数或遍历位置。
    ++g_progress_frame_count;
    // 更新 frame[0] 对应的协议帧字段。
    frame[0] = COMMUNICATION_FRAME_HEADER_1;
    // 更新 frame[1] 对应的协议帧字段。
    frame[1] = COMMUNICATION_FRAME_HEADER_2;
    // 更新 frame[2] 对应的协议帧字段。
    frame[2] = COMMUNICATION_COMMAND_PROGRESS;
    // 更新 frame[3] 对应的协议帧字段。
    frame[3] = 2U;
    // 更新 frame[4] 对应的协议帧字段。
    frame[4] = (uint8_t)(progress & 0xFFU);
    // 更新 frame[5] 对应的协议帧字段。
    frame[5] = (uint8_t)(progress >> 8U);
    // 更新 frame[6] 对应的协议帧字段。
    frame[6] = frame[2] ^ frame[3] ^ frame[4] ^ frame[5];
    // 调用 BSP_UART_RpiWrite，更新或发送对应数据。
    BSP_UART_RpiWrite(frame, (uint16_t)sizeof(frame));
}

/**
 * @brief 将有效树莓派视觉帧提交给云台闭环
 * @param parser 已完成解析的视觉协议帧
 */
static void COMMUNICATION_HandleVision(
    const COMMUNICATION_Parser_t *parser)
{
    // 视觉帧载荷固定为有效标志和四个小端有符号坐标。
    if ((parser->command != COMMUNICATION_COMMAND_VISION) ||
        (parser->length != 9U))
    {
        // 更新对应的累计计数或遍历位置。
        ++g_protocol_error_count;
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 保存原始视觉量用于调试，并把激光坐标提交给云台控制层。
    g_vision_ok = parser->payload[0];
    // 调用 COMMUNICATION_ReadInt16，读取当前反馈或状态。
    g_vision_laser_x = COMMUNICATION_ReadInt16(&parser->payload[1]);
    // 调用 COMMUNICATION_ReadInt16，读取当前反馈或状态。
    g_vision_laser_y = COMMUNICATION_ReadInt16(&parser->payload[3]);
    // 调用 COMMUNICATION_ReadInt16，读取当前反馈或状态。
    g_vision_center_x = COMMUNICATION_ReadInt16(&parser->payload[5]);
    // 调用 COMMUNICATION_ReadInt16，读取当前反馈或状态。
    g_vision_center_y = COMMUNICATION_ReadInt16(&parser->payload[7]);
    // 更新对应的累计计数或遍历位置。
    ++g_vision_frame_count;
    // 调用 GIMBAL_AxisSetVisionTarget，更新或发送对应数据。
    GIMBAL_AxisSetVisionTarget(g_vision_ok != 0U,
        g_vision_laser_x, g_vision_laser_y);
}

/**
 * @brief 初始化云台通信业务状态
 */
void COMMUNICATION_ServiceInit(void)
{
    // 调用 COMMUNICATION_ResetParser，复位对应模块的历史状态。
    COMMUNICATION_ResetParser(&s_mspa_parser);
    // 调用 COMMUNICATION_ResetParser，复位对应模块的历史状态。
    COMMUNICATION_ResetParser(&s_rpi_parser);
    // 更新 g_vehicle_progress 对应的进度状态。
    g_vehicle_progress = 0U;
    // 更新 g_progress_frame_count 对应的协议帧字段。
    g_progress_frame_count = 0U;
    // 更新 g_vision_frame_count 对应的协议帧字段。
    g_vision_frame_count = 0U;
    // 更新 g_protocol_error_count 对应的累计计数。
    g_protocol_error_count = 0U;
    // 更新 g_vision_ok 对应的本步骤的运行数据。
    g_vision_ok = 0U;
    // 更新 g_vision_laser_x 对应的本步骤的运行数据。
    g_vision_laser_x = 0;
    // 更新 g_vision_laser_y 对应的本步骤的运行数据。
    g_vision_laser_y = 0;
    // 更新 g_vision_center_x 对应的本步骤的运行数据。
    g_vision_center_x = 0;
    // 更新 g_vision_center_y 对应的本步骤的运行数据。
    g_vision_center_y = 0;
}

/**
 * @brief 处理已接收的云台通信业务数据
 */
void COMMUNICATION_ServiceProcess(void)
{
    // 定义本步骤需要的局部数据并完成初始化。
    uint8_t data;

    // UART2 只接收底盘进度，并把完整有效帧转发给树莓派。
    while (BSP_UART_MspaReadByte(&data))
    {
        // 判断 COMMUNICATION_ParseByte(&s_mspa_parser, data) 是否成立，并选择对应处理分支。
        if (COMMUNICATION_ParseByte(&s_mspa_parser, data))
        {
            // 调用 COMMUNICATION_HandleProgress，完成当前步骤的业务处理。
            COMMUNICATION_HandleProgress(&s_mspa_parser);
        }
    }

    // UART3 接收树莓派视觉目标，并提交给云台位置闭环。
    while (BSP_UART_RpiReadByte(&data))
    {
        // 判断 COMMUNICATION_ParseByte(&s_rpi_parser, data) 是否成立，并选择对应处理分支。
        if (COMMUNICATION_ParseByte(&s_rpi_parser, data))
        {
            // 调用 COMMUNICATION_HandleVision，完成当前步骤的业务处理。
            COMMUNICATION_HandleVision(&s_rpi_parser);
        }
    }
}
