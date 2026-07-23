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

// 更新 _Static_assert(sizeof(float) 对应的本步骤的运行数据。
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
    // 定义本步骤需要的局部数据并完成初始化。
    size_t prefix_length;
    // 定义本步骤需要的局部数据并完成初始化。
    char *parse_end;
    // 定义本步骤需要的局部数据并完成初始化。
    const char *number_start;

    // 任一输入地址为空时拒绝解析，避免调试命令造成非法访问。
    if ((command == 0) || (prefix == 0) || (value == 0))
    {
        // 返回失败状态，表示当前条件尚未满足。
        return false;
    }

    // 命令前缀必须完整匹配，随后只解析前缀后的浮点参数。
    prefix_length = strlen(prefix);
    // 检查相关输入、计数或对象状态是否有效。
    if (strncmp(command, prefix, prefix_length) != 0)
    {
        // 返回失败状态，表示当前条件尚未满足。
        return false;
    }

    // 更新 number_start 对应的本步骤的运行数据。
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
    // 定义本步骤需要的局部数据并完成初始化。
    WHEEL_SpeedTelemetry_t telemetry;
    // 定义本步骤需要的局部数据并完成初始化。
    float value;

    // 执行命令前读取当前参数，单项修改时保留其余设定。
    WHEEL_SpeedControlGetTelemetry(&telemetry);

    // STOP关闭循迹、速度闭环和驱动总使能。
    if (strcmp(command, "STOP") == 0)
    {
        // 更新 g_line_follow_enabled 对应的有效或使能状态。
        g_line_follow_enabled = false;
        // 调用 CHASSIS_SetLineFollowEnabled，更新或发送对应数据。
        CHASSIS_SetLineFollowEnabled(false);
        // 调用 CHASSIS_SetWheelSpeedTarget，更新或发送对应数据。
        CHASSIS_SetWheelSpeedTarget(0.0f, 0.0f);
        // 调用 CHASSIS_SetSpeedClosedLoopEnabled，更新或发送对应数据。
        CHASSIS_SetSpeedClosedLoopEnabled(false);
        // 调用 CHASSIS_SetEnabled，更新或发送对应数据。
        CHASSIS_SetEnabled(false);
    }
    // 前一条件不成立时继续判断当前条件。
    else if (VOFA_ParseSingleValue(command, "EN=", &value))
    {
        // EN命令进入独立轮速调试模式，不允许循迹外环同时接管目标。
        const bool enable = value != 0.0f;
        // 更新 g_line_follow_enabled 对应的有效或使能状态。
        g_line_follow_enabled = false;
        // 调用 CHASSIS_SetLineFollowEnabled，更新或发送对应数据。
        CHASSIS_SetLineFollowEnabled(false);
        // 调用 CHASSIS_SetSpeedClosedLoopEnabled，更新或发送对应数据。
        CHASSIS_SetSpeedClosedLoopEnabled(enable);
        // 调用 CHASSIS_SetEnabled，更新或发送对应数据。
        CHASSIS_SetEnabled(enable);
    }
    // 前一条件不成立时继续判断当前条件。
    else if (VOFA_ParseSingleValue(command, "T=", &value))
    {
        // T命令同时设置左右轮目标速度。
        CHASSIS_SetWheelSpeedTarget(value, value);
    }
    // 前一条件不成立时继续判断当前条件。
    else if (VOFA_ParseSingleValue(command, "TL=", &value))
    {
        // TL命令只修改左轮目标并保留右轮目标。
        CHASSIS_SetWheelSpeedTarget(value, telemetry.right_target_cps);
    }
    // 前一条件不成立时继续判断当前条件。
    else if (VOFA_ParseSingleValue(command, "TR=", &value))
    {
        // TR命令只修改右轮目标并保留左轮目标。
        CHASSIS_SetWheelSpeedTarget(telemetry.left_target_cps, value);
    }
    // 前一条件不成立时继续判断当前条件。
    else if (VOFA_ParseSingleValue(command, "KP=", &value))
    {
        // 调用 CHASSIS_SetWheelSpeedPid，更新或发送对应数据。
        CHASSIS_SetWheelSpeedPid(value, telemetry.ki, telemetry.kd);
    }
    // 前一条件不成立时继续判断当前条件。
    else if (VOFA_ParseSingleValue(command, "KI=", &value))
    {
        // 调用 CHASSIS_SetWheelSpeedPid，更新或发送对应数据。
        CHASSIS_SetWheelSpeedPid(telemetry.kp, value, telemetry.kd);
    }
    // 前一条件不成立时继续判断当前条件。
    else if (VOFA_ParseSingleValue(command, "KD=", &value))
    {
        // 调用 CHASSIS_SetWheelSpeedPid，更新或发送对应数据。
        CHASSIS_SetWheelSpeedPid(telemetry.kp, telemetry.ki, value);
    }
    // 前一条件不成立时继续判断当前条件。
    else if (VOFA_ParseSingleValue(command, "FF=", &value))
    {
        // 调用 CHASSIS_SetWheelSpeedFeedforward，更新或发送对应数据。
        CHASSIS_SetWheelSpeedFeedforward(value);
    }
}

/**
 * @brief 从 UART3 环形缓冲区收集并解析文本命令
 */
static void VOFA_ProcessReceive(void)
{
    // 定义本步骤需要的局部数据并完成初始化。
    uint8_t data;

    // 按行接收文本命令，回车或换行表示一条命令结束。
    while (BSP_UART_VofaReadByte(&data))
    {
        // 判断 (data == '\r') || (data == '\n') 是否成立，并选择对应处理分支。
        if ((data == '\r') || (data == '\n'))
        {
            // 判断 s_command_length > 0U 是否成立，并选择对应处理分支。
            if (s_command_length > 0U)
            {
                // 更新 s_command_buffer[s_command_length] 对应的本步骤的运行数据。
                s_command_buffer[s_command_length] = '\0';
                // 调用 VOFA_ExecuteCommand，完成当前步骤的业务处理。
                VOFA_ExecuteCommand(s_command_buffer);
                // 更新 s_command_length 对应的本步骤的运行数据。
                s_command_length = 0U;
            }
        }
        // 前一条件不成立时继续判断当前条件。
        else if (s_command_length < (VOFA_COMMAND_BUFFER_SIZE - 1U))
        {
            // 更新 s_command_buffer[s_command_length] 对应的本步骤的运行数据。
            s_command_buffer[s_command_length] = (char)data;
            // 更新对应的累计计数或遍历位置。
            ++s_command_length;
        }
        // 前述条件均不成立时执行备用处理。
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
    // 定义本步骤需要的局部数据并完成初始化。
    float channels[VOFA_CHANNEL_COUNT];

    // 空遥测地址不发送，避免输出无效波形帧。
    if (telemetry == 0)
    {
        // 返回本次计算或查询得到的结果。
        return;
    }

    // 通道顺序固定，保证VOFA+工程中的曲线含义不会随运行改变。
    channels[0] = telemetry->left_target_cps;
    // 更新 channels[1] 对应的本步骤的运行数据。
    channels[1] = telemetry->left_feedback_cps;
    // 更新 channels[2] 对应的本步骤的运行数据。
    channels[2] = telemetry->left_output;
    // 更新 channels[3] 对应的本步骤的运行数据。
    channels[3] = telemetry->right_target_cps;
    // 更新 channels[4] 对应的本步骤的运行数据。
    channels[4] = telemetry->right_feedback_cps;
    // 更新 channels[5] 对应的本步骤的运行数据。
    channels[5] = telemetry->right_output;
    // 更新 channels[6] 对应的本步骤的运行数据。
    channels[6] = telemetry->kp;
    // 更新 channels[7] 对应的本步骤的运行数据。
    channels[7] = telemetry->ki;
    // 更新 channels[8] 对应的本步骤的运行数据。
    channels[8] = telemetry->kd;
    // 更新 channels[9] 对应的本步骤的运行数据。
    channels[9] = telemetry->feedforward_gain;
    // 更新 channels[10] 对应的本步骤的运行数据。
    channels[10] = telemetry->left_native_delta;
    // 更新 channels[11] 对应的本步骤的运行数据。
    channels[11] = telemetry->right_native_delta;
    // 更新 channels[12] 对应的本步骤的运行数据。
    channels[12] = telemetry->elapsed_ticks;

    // JustFloat数据区后追加固定四字节帧尾。
    BSP_UART_VofaWrite((const uint8_t *)channels, sizeof(channels));
    // 调用 BSP_UART_VofaWrite，更新或发送对应数据。
    BSP_UART_VofaWrite(s_justfloat_tail, sizeof(s_justfloat_tail));
}

/**
 * @brief 初始化 VOFA+ 调参服务
 */
void VOFA_ServiceInit(void)
{
    // 更新 s_command_length 对应的本步骤的运行数据。
    s_command_length = 0U;
    // 更新 s_last_send_update_count 对应的累计计数。
    s_last_send_update_count = 0U;
}

/**
 * @brief 处理 VOFA+ 命令并按 50Hz 上传 JustFloat 波形
 */
void VOFA_ServiceProcess(void)
{
    // 定义本步骤需要的局部数据并完成初始化。
    WHEEL_SpeedTelemetry_t telemetry;

    // 每次调度先处理下行命令，使新参数立即用于后续控制周期。
    VOFA_ProcessReceive();
    // 调用 WHEEL_SpeedControlGetTelemetry，读取当前反馈或状态。
    WHEEL_SpeedControlGetTelemetry(&telemetry);
    // 按轮速更新计数分频到50Hz上传，避免占满调试串口带宽。
    if ((uint32_t)(telemetry.update_count - s_last_send_update_count) >=
        VOFA_SEND_INTERVAL_TICKS)
    {
        // 更新 s_last_send_update_count 对应的累计计数。
        s_last_send_update_count = telemetry.update_count;
        // 调用 VOFA_SendJustFloat，更新或发送对应数据。
        VOFA_SendJustFloat(&telemetry);
    }
}
