/**
 * @file        gimbal_task.c
 * @author      JerryZheng
 * @brief       云台周期业务服务
 * @date        2026-07-21
 */

#include "gimbal_task.h"
#include "gimbal_axis.h"
#include "imu_follow_service.h"
#include "mt6816.h"
#include "wt901.h"
#include "bsp_timer.h"

/** @brief Live Watch：云台完整诊断状态。 */
volatile GIMBAL_DebugStatus_t g_gimbal_debug = { 0 };

/**
 * @brief 刷新云台完整 Live Watch 诊断状态
 */
static void GIMBAL_TaskUpdateDebugStatus(void)
{
    GIMBAL_AxisTelemetry_t axis;
    MT6816_Status_t yaw_encoder;
    MT6816_Status_t pitch_encoder;

    GIMBAL_AxisGetTelemetry(&axis);
    MT6816_GetStatus(MT6816_AXIS_YAW, &yaw_encoder);
    MT6816_GetStatus(MT6816_AXIS_PITCH, &pitch_encoder);

    // 汇总姿态源、编码器和控制器状态，避免 Live Watch 分散读取多个模块。
    g_gimbal_debug.wt901_yaw_deg = g_wt901_angle.yaw;
    g_gimbal_debug.wt901_pitch_deg = g_wt901_angle.pitch;
    g_gimbal_debug.wt901_angle_frames =
        g_wt901_angle_update_count;
    g_gimbal_debug.follow_active = g_wt901_follow_active;
    g_gimbal_debug.feedback_ready =
        GIMBAL_AxisIsReady() ? 1U : 0U;
    g_gimbal_debug.follow_yaw_deg = g_wt901_follow_yaw_deg;
    g_gimbal_debug.follow_pitch_deg = g_wt901_follow_pitch_deg;
    g_gimbal_debug.follow_age_ms = g_wt901_follow_age_ms;
    g_gimbal_debug.yaw_pwm_valid =
        yaw_encoder.pwm_valid ? 1U : 0U;
    g_gimbal_debug.yaw_synchronized =
        yaw_encoder.synchronized ? 1U : 0U;
    g_gimbal_debug.yaw_pwm_high_ticks =
        yaw_encoder.pwm_high_ticks;
    g_gimbal_debug.yaw_pwm_period_ticks =
        yaw_encoder.pwm_period_ticks;
    g_gimbal_debug.yaw_pwm_frames =
        yaw_encoder.pwm_frame_count;
    g_gimbal_debug.pitch_pwm_valid =
        pitch_encoder.pwm_valid ? 1U : 0U;
    g_gimbal_debug.pitch_synchronized =
        pitch_encoder.synchronized ? 1U : 0U;
    g_gimbal_debug.pitch_pwm_high_ticks =
        pitch_encoder.pwm_high_ticks;
    g_gimbal_debug.pitch_pwm_period_ticks =
        pitch_encoder.pwm_period_ticks;
    g_gimbal_debug.pitch_pwm_frames =
        pitch_encoder.pwm_frame_count;
    g_gimbal_debug.yaw_position = axis.yaw_position;
    g_gimbal_debug.pitch_position = axis.pitch_position;
    g_gimbal_debug.yaw_target = axis.yaw_target;
    g_gimbal_debug.pitch_target = axis.pitch_target;
    g_gimbal_debug.yaw_step_hz = axis.yaw_step_hz;
    g_gimbal_debug.pitch_step_hz = axis.pitch_step_hz;
}

/**
 * @brief 初始化云台周期业务
 */
void GIMBAL_TaskInit(void)
{
    // 先初始化云台闭环，确保 WT901 数据到达前驱动器保持在安全状态。
    GIMBAL_AxisInit();

    // WT901 初始化成功后才开放串口数据接收。
    if (WT901_Init() == WT901_OK)
    {
        (void)WT901_StartReceive();
    }
    IMU_FollowServiceInit();
    GIMBAL_TaskUpdateDebugStatus();
}

/**
 * @brief 在 1ms 节拍到达时处理姿态数据与云台控制业务
 */
void GIMBAL_TaskProcess(void)
{
    const uint32_t elapsed_ticks = BSP_Timer_TakeGimbalTicks();

    // 每次调度清空已接收的完整帧，只保留最新姿态供跟随业务使用。
    while (WT901_AnalyzeData())
    {
    }

    if (elapsed_ticks == 0U)
    {
        return;
    }
    // 先由业务层生成目标，再由控制层根据编码器反馈输出 STEP。
    IMU_FollowServiceProcess(elapsed_ticks);
    GIMBAL_AxisUpdate(elapsed_ticks);
    GIMBAL_TaskUpdateDebugStatus();
}
