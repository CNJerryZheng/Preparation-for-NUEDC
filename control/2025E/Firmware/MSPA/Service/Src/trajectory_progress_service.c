/**
 * @file        trajectory_progress_service.c
 * @author      JerryZheng
 * @brief       小车轨迹进度计算与上报服务实现
 * @date        2026-07-23
 */

#include "trajectory_progress_service.h"
#include "trajectory_progress_config.h"
#include "bsp_uart.h"
#include "line_control_config.h"
#include "mg513x.h"
#include "wheel_speed_control.h"

#define TRAJECTORY_FRAME_HEADER_1 (0xAAU)
#define TRAJECTORY_FRAME_HEADER_2 (0x55U)
#define TRAJECTORY_COMMAND_PROGRESS (0x01U)
#define TRAJECTORY_PROGRESS_PAYLOAD_LENGTH (2U)

/** @brief 当前运动周期累计的等效x1霍尔计数。 */
static float s_cycle_count = 0.0f;
/** @brief 上次读取到的左轮原生霍尔累计值。 */
static int32_t s_last_left_native_count = 0;
/** @brief 上次读取到的右轮原生霍尔累计值。 */
static int32_t s_last_right_native_count = 0;
/** @brief 上次发送进度时的底盘控制更新计数。 */
static uint32_t s_last_send_update_count = 0U;
/** @brief 是否需要先发送一次100%完成帧。 */
static bool s_cycle_complete_pending = false;

volatile uint16_t g_trajectory_progress = 0U;
volatile uint32_t g_trajectory_progress_frame_count = 0U;

/**
 * @brief 计算一个运动周期对应的等效x1霍尔计数
 * @return float 一个完整周期的等效x1计数
 */
static float TRAJECTORY_GetCycleCount(void)
{
    return LINE_CONTROL_MPS_TO_CPS(
        TRAJECTORY_PROGRESS_CYCLE_DISTANCE_M);
}

/**
 * @brief 计算有符号32位计数的绝对值
 * @param value 原始有符号计数
 * @return float 非负计数
 */
static float TRAJECTORY_AbsCount(int32_t value)
{
    return (value < 0) ? -(float)value : (float)value;
}

/**
 * @brief 根据当前累计计数刷新0~10000进度值
 */
static void TRAJECTORY_UpdateProgressValue(void)
{
    const float cycle_count = TRAJECTORY_GetCycleCount();
    float scaled_progress;

    // 跨越整周期后先保留一帧100%，避免接收端只看到回绕后的低进度。
    if (s_cycle_complete_pending)
    {
        g_trajectory_progress = TRAJECTORY_PROGRESS_FULL_SCALE;
        return;
    }
    // 周期距离配置无效时输出零进度，避免浮点除零。
    if (cycle_count <= 0.0f)
    {
        g_trajectory_progress = 0U;
        return;
    }

    // 将当前等效行程线性映射到协议规定的0～10000范围。
    scaled_progress = s_cycle_count *
        (float)TRAJECTORY_PROGRESS_FULL_SCALE / cycle_count;
    if (scaled_progress > (float)TRAJECTORY_PROGRESS_FULL_SCALE)
    {
        scaled_progress = (float)TRAJECTORY_PROGRESS_FULL_SCALE;
    }
    g_trajectory_progress = (uint16_t)scaled_progress;
}

/**
 * @brief 按文档格式发送一帧轨迹进度
 */
static void TRAJECTORY_SendProgressFrame(void)
{
    uint8_t frame[7];

    // 按AA 55、命令、长度、载荷、异或校验的固定格式组帧。
    frame[0] = TRAJECTORY_FRAME_HEADER_1;
    frame[1] = TRAJECTORY_FRAME_HEADER_2;
    frame[2] = TRAJECTORY_COMMAND_PROGRESS;
    frame[3] = TRAJECTORY_PROGRESS_PAYLOAD_LENGTH;
    frame[4] = (uint8_t)(g_trajectory_progress & 0xFFU);
    frame[5] = (uint8_t)(g_trajectory_progress >> 8U);
    frame[6] = frame[2] ^ frame[3] ^ frame[4] ^ frame[5];
    BSP_UART_MspbWrite(frame, (uint16_t)sizeof(frame));
    ++g_trajectory_progress_frame_count;
}

/** @copydoc TRAJECTORY_ProgressServiceInit */
void TRAJECTORY_ProgressServiceInit(void)
{
    WHEEL_SpeedTelemetry_t telemetry;

    // 从当前霍尔累计值开始计程，防止把上电前的计数差当作有效路程。
    s_last_left_native_count =
        MG513X_GetHallCount(MG513X_MOTOR_LEFT);
    s_last_right_native_count =
        MG513X_GetHallCount(MG513X_MOTOR_RIGHT);
    WHEEL_SpeedControlGetTelemetry(&telemetry);
    s_last_send_update_count = telemetry.update_count;
    s_cycle_count = 0.0f;
    s_cycle_complete_pending = false;
    g_trajectory_progress = 0U;
    g_trajectory_progress_frame_count = 0U;
}

/** @copydoc TRAJECTORY_ProgressReset */
void TRAJECTORY_ProgressReset(void)
{
    s_last_left_native_count =
        MG513X_GetHallCount(MG513X_MOTOR_LEFT);
    s_last_right_native_count =
        MG513X_GetHallCount(MG513X_MOTOR_RIGHT);
    s_cycle_count = 0.0f;
    s_cycle_complete_pending = false;
    g_trajectory_progress = 0U;
}

/** @copydoc TRAJECTORY_ProgressServiceProcess */
void TRAJECTORY_ProgressServiceProcess(void)
{
    const int32_t left_native_count =
        MG513X_GetHallCount(MG513X_MOTOR_LEFT);
    const int32_t right_native_count =
        MG513X_GetHallCount(MG513X_MOTOR_RIGHT);
    // 使用无符号差再转回有符号数，保证32位累计计数回绕时差值正确。
    const int32_t left_native_delta = (int32_t)(
        (uint32_t)left_native_count - (uint32_t)s_last_left_native_count);
    const int32_t right_native_delta = (int32_t)(
        (uint32_t)right_native_count - (uint32_t)s_last_right_native_count);
    // 不论前进或倒车都按实际轮路程累计，并换算回统一的x1霍尔计数。
    const float left_x1_delta = TRAJECTORY_AbsCount(left_native_delta) /
        (float)MG513X_GetHallDecodeMultiplier(MG513X_MOTOR_LEFT);
    const float right_x1_delta = TRAJECTORY_AbsCount(right_native_delta) /
        (float)MG513X_GetHallDecodeMultiplier(MG513X_MOTOR_RIGHT);
    const float cycle_count = TRAJECTORY_GetCycleCount();
    WHEEL_SpeedTelemetry_t telemetry;

    s_last_left_native_count = left_native_count;
    s_last_right_native_count = right_native_count;
    // 双轮路程取平均值，减小转弯时单侧轮速差对整车进度的影响。
    s_cycle_count += (left_x1_delta + right_x1_delta) * 0.5f;

    // 支持一次更新跨越多个周期，并标记下一帧先上报100%。
    if ((cycle_count > 0.0f) && (s_cycle_count >= cycle_count))
    {
        do
        {
            s_cycle_count -= cycle_count;
        } while (s_cycle_count >= cycle_count);
        s_cycle_complete_pending = true;
    }

    WHEEL_SpeedControlGetTelemetry(&telemetry);
    // 发送频率由轮速控制更新计数分频，避免依赖主循环执行速度。
    if ((telemetry.update_count - s_last_send_update_count) <
        TRAJECTORY_PROGRESS_SEND_INTERVAL_TICKS)
    {
        return;
    }

    s_last_send_update_count = telemetry.update_count;
    TRAJECTORY_UpdateProgressValue();
    TRAJECTORY_SendProgressFrame();
    // 100%帧发出后清除完成标志，下一帧从回绕后的真实进度继续。
    if (s_cycle_complete_pending)
    {
        s_cycle_complete_pending = false;
        TRAJECTORY_UpdateProgressValue();
    }
}
