/**
 * @file        vofa_service.c
 * @author      JerryZheng
 * @brief       VOFA+ 轮速 PID 调参服务实现
 * @date        2026-07-22
 */

#include "vofa_service.h"
#include "bsp_uart.h"
#include "chassis_control.h"
#include "chassis_task.h"
#include "wheel_speed_control.h"

#include <stdlib.h>
#include <string.h>

#define VOFA_COMMAND_BUFFER_SIZE (64U)
#define VOFA_CHANNEL_COUNT (13U)
#define VOFA_SEND_INTERVAL_TICKS (2U)

/** @brief VOFA+ JustFloat 帧尾。 */
static const uint8_t s_justfloat_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
/** @brief VOFA+ 下行文本命令缓冲区。 */
static char s_command_buffer[VOFA_COMMAND_BUFFER_SIZE];
/** @brief VOFA+ 下行文本命令当前长度。 */
static uint16_t s_command_length = 0U;
/** @brief 上一次上传波形时的 10ms 控制计数。 */
static uint32_t s_last_send_update_count = 0U;

_Static_assert(sizeof(float) == 4U, "VOFA+ JustFloat requires 32-bit float");

/**
 * @brief 解析等号后的单个浮点参数
 * @param command 完整命令字符串
 * @param prefix 命令前缀
 * @param value 浮点结果输出地址
 * @retval true 命令前缀匹配且成功取得参数
 * @retval false 命令不匹配或输出地址为空
 */
static bool VOFA_ParseSingleValue(const char *command,
    const char *prefix, float *value)
{
    size_t prefix_length;
    char *parse_end;
    const char *number_start;

    // 任一输入地址为空时拒绝解析，避免调试命令造成非法访问。
    if ((command == 0) || (prefix == 0) || (value == 0))
    {
        return false;
    }

    // 命令前缀必须完整匹配，随后只解析前缀后的浮点参数。
    prefix_length = strlen(prefix);
    if (strncmp(command, prefix, prefix_length) != 0)
    {
        return false;
    }

    number_start = command + prefix_length;
    *value = strtof(number_start, &parse_end);
    return parse_end != number_start;
}

/**
 * @brief 执行一条 VOFA+ 下行调参命令
 * @param command 以空字符结尾的命令字符串
 */
static void VOFA_ExecuteCommand(const char *command)
{
    WHEEL_SpeedTelemetry_t telemetry;
    float value;

    // 执行命令前读取当前参数，单项修改时保留其余设定。
    WHEEL_SpeedControlGetTelemetry(&telemetry);

    // STOP关闭循迹、速度闭环和驱动总使能。
    if (strcmp(command, "STOP") == 0)
    {
        g_line_follow_enabled = false;
        CHASSIS_SetLineFollowEnabled(false);
        CHASSIS_SetWheelSpeedTarget(0.0f, 0.0f);
        CHASSIS_SetSpeedClosedLoopEnabled(false);
        CHASSIS_SetEnabled(false);
    }
    else if (VOFA_ParseSingleValue(command, "EN=", &value))
    {
        // EN命令进入独立轮速调试模式，不允许循迹外环同时接管目标。
        const bool enable = value != 0.0f;
        g_line_follow_enabled = false;
        CHASSIS_SetLineFollowEnabled(false);
        CHASSIS_SetSpeedClosedLoopEnabled(enable);
        CHASSIS_SetEnabled(enable);
    }
    else if (VOFA_ParseSingleValue(command, "T=", &value))
    {
        // T命令同时设置左右轮目标速度。
        CHASSIS_SetWheelSpeedTarget(value, value);
    }
    else if (VOFA_ParseSingleValue(command, "TL=", &value))
    {
        // TL命令只修改左轮目标并保留右轮目标。
        CHASSIS_SetWheelSpeedTarget(value, telemetry.right_target_cps);
    }
    else if (VOFA_ParseSingleValue(command, "TR=", &value))
    {
        // TR命令只修改右轮目标并保留左轮目标。
        CHASSIS_SetWheelSpeedTarget(telemetry.left_target_cps, value);
    }
    else if (VOFA_ParseSingleValue(command, "KP=", &value))
    {
        CHASSIS_SetWheelSpeedPid(value, telemetry.ki, telemetry.kd);
    }
    else if (VOFA_ParseSingleValue(command, "KI=", &value))
    {
        CHASSIS_SetWheelSpeedPid(telemetry.kp, value, telemetry.kd);
    }
    else if (VOFA_ParseSingleValue(command, "KD=", &value))
    {
        CHASSIS_SetWheelSpeedPid(telemetry.kp, telemetry.ki, value);
    }
    else if (VOFA_ParseSingleValue(command, "FF=", &value))
    {
        CHASSIS_SetWheelSpeedFeedforward(value);
    }
}

/**
 * @brief 从 UART3 环形缓冲区收集并解析文本命令
 */
static void VOFA_ProcessReceive(void)
{
    uint8_t data;

    // 按行接收文本命令，回车或换行表示一条命令结束。
    while (BSP_UART_VofaReadByte(&data))
    {
        if ((data == '\r') || (data == '\n'))
        {
            if (s_command_length > 0U)
            {
                s_command_buffer[s_command_length] = '\0';
                VOFA_ExecuteCommand(s_command_buffer);
                s_command_length = 0U;
            }
        }
        else if (s_command_length < (VOFA_COMMAND_BUFFER_SIZE - 1U))
        {
            s_command_buffer[s_command_length] = (char)data;
            ++s_command_length;
        }
        else
        {
            /* 超长命令直接丢弃，等待下一行重新同步。 */
            s_command_length = 0U;
        }
    }
}

/**
 * @brief 按 JustFloat 格式发送一帧轮速 PID 波形
 * @param telemetry 当前双轮速度闭环遥测数据
 */
static void VOFA_SendJustFloat(const WHEEL_SpeedTelemetry_t *telemetry)
{
    float channels[VOFA_CHANNEL_COUNT];

    // 空遥测地址不发送，避免输出无效波形帧。
    if (telemetry == 0)
    {
        return;
    }

    // 通道顺序固定，保证VOFA+工程中的曲线含义不会随运行改变。
    channels[0] = telemetry->left_target_cps;
    channels[1] = telemetry->left_feedback_cps;
    channels[2] = telemetry->left_output;
    channels[3] = telemetry->right_target_cps;
    channels[4] = telemetry->right_feedback_cps;
    channels[5] = telemetry->right_output;
    channels[6] = telemetry->kp;
    channels[7] = telemetry->ki;
    channels[8] = telemetry->kd;
    channels[9] = telemetry->feedforward_gain;
    channels[10] = telemetry->left_native_delta;
    channels[11] = telemetry->right_native_delta;
    channels[12] = telemetry->elapsed_ticks;

    // JustFloat数据区后追加固定四字节帧尾。
    BSP_UART_VofaWrite((const uint8_t *)channels, sizeof(channels));
    BSP_UART_VofaWrite(s_justfloat_tail, sizeof(s_justfloat_tail));
}

/**
 * @brief 初始化 VOFA+ 调参服务
 */
void VOFA_ServiceInit(void)
{
    s_command_length = 0U;
    s_last_send_update_count = 0U;
}

/**
 * @brief 处理 VOFA+ 命令并按 50Hz 上传 JustFloat 波形
 */
void VOFA_ServiceProcess(void)
{
    WHEEL_SpeedTelemetry_t telemetry;

    // 每次调度先处理下行命令，使新参数立即用于后续控制周期。
    VOFA_ProcessReceive();
    WHEEL_SpeedControlGetTelemetry(&telemetry);
    // 按轮速更新计数分频到50Hz上传，避免占满调试串口带宽。
    if ((uint32_t)(telemetry.update_count - s_last_send_update_count) >=
        VOFA_SEND_INTERVAL_TICKS)
    {
        s_last_send_update_count = telemetry.update_count;
        VOFA_SendJustFloat(&telemetry);
    }
}
