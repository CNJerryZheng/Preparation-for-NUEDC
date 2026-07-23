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
    // 定义本步骤需要的局部数据并完成初始化。
    GIMBAL_AxisTelemetry_t axis;
    // 定义本步骤需要的局部数据并完成初始化。
    MT6816_Status_t yaw_encoder;
    // 定义本步骤需要的局部数据并完成初始化。
    MT6816_Status_t pitch_encoder;

    // 调用 GIMBAL_AxisGetTelemetry，读取当前反馈或状态。
    GIMBAL_AxisGetTelemetry(&axis);
    // 调用 MT6816_GetStatus，读取当前反馈或状态。
    MT6816_GetStatus(MT6816_AXIS_YAW, &yaw_encoder);
    // 调用 MT6816_GetStatus，读取当前反馈或状态。
    MT6816_GetStatus(MT6816_AXIS_PITCH, &pitch_encoder);

    // 汇总姿态源、编码器和控制器状态，避免 Live Watch 分散读取多个模块。
    g_gimbal_debug.wt901_yaw_deg = g_wt901_angle.yaw;
    // 更新 g_gimbal_debug.wt901_pitch_deg 对应的本步骤的运行数据。
    g_gimbal_debug.wt901_pitch_deg = g_wt901_angle.pitch;
    // 更新 g_gimbal_debug.wt901_angle_frames 对应的协议帧字段。
    g_gimbal_debug.wt901_angle_frames =
        g_wt901_angle_update_count;
    // 更新 g_gimbal_debug.follow_active 对应的有效或使能状态。
    g_gimbal_debug.follow_active = g_wt901_follow_active;
    // 调用 GIMBAL_AxisIsReady，读取当前反馈或状态。
    g_gimbal_debug.feedback_ready =
        GIMBAL_AxisIsReady() ? 1U : 0U;
    // 更新 g_gimbal_debug.follow_yaw_deg 对应的本步骤的运行数据。
    g_gimbal_debug.follow_yaw_deg = g_wt901_follow_yaw_deg;
    // 更新 g_gimbal_debug.follow_pitch_deg 对应的本步骤的运行数据。
    g_gimbal_debug.follow_pitch_deg = g_wt901_follow_pitch_deg;
    // 更新 g_gimbal_debug.follow_age_ms 对应的时间状态。
    g_gimbal_debug.follow_age_ms = g_wt901_follow_age_ms;
    // 更新 g_gimbal_debug.yaw_pwm_valid 对应的有效或使能状态。
    g_gimbal_debug.yaw_pwm_valid =
        yaw_encoder.pwm_valid ? 1U : 0U;
    // 更新 g_gimbal_debug.yaw_synchronized 对应的本步骤的运行数据。
    g_gimbal_debug.yaw_synchronized =
        yaw_encoder.synchronized ? 1U : 0U;
    // 更新 g_gimbal_debug.yaw_pwm_high_ticks 对应的本步骤的运行数据。
    g_gimbal_debug.yaw_pwm_high_ticks =
        yaw_encoder.pwm_high_ticks;
    // 更新 g_gimbal_debug.yaw_pwm_period_ticks 对应的本步骤的运行数据。
    g_gimbal_debug.yaw_pwm_period_ticks =
        yaw_encoder.pwm_period_ticks;
    // 更新 g_gimbal_debug.yaw_pwm_frames 对应的协议帧字段。
    g_gimbal_debug.yaw_pwm_frames =
        yaw_encoder.pwm_frame_count;
    // 更新 g_gimbal_debug.pitch_pwm_valid 对应的有效或使能状态。
    g_gimbal_debug.pitch_pwm_valid =
        pitch_encoder.pwm_valid ? 1U : 0U;
    // 更新 g_gimbal_debug.pitch_synchronized 对应的本步骤的运行数据。
    g_gimbal_debug.pitch_synchronized =
        pitch_encoder.synchronized ? 1U : 0U;
    // 更新 g_gimbal_debug.pitch_pwm_high_ticks 对应的本步骤的运行数据。
    g_gimbal_debug.pitch_pwm_high_ticks =
        pitch_encoder.pwm_high_ticks;
    // 更新 g_gimbal_debug.pitch_pwm_period_ticks 对应的本步骤的运行数据。
    g_gimbal_debug.pitch_pwm_period_ticks =
        pitch_encoder.pwm_period_ticks;
    // 更新 g_gimbal_debug.pitch_pwm_frames 对应的协议帧字段。
    g_gimbal_debug.pitch_pwm_frames =
        pitch_encoder.pwm_frame_count;
    // 更新 g_gimbal_debug.yaw_position 对应的位置状态。
    g_gimbal_debug.yaw_position = axis.yaw_position;
    // 更新 g_gimbal_debug.pitch_position 对应的位置状态。
    g_gimbal_debug.pitch_position = axis.pitch_position;
    // 更新 g_gimbal_debug.yaw_target 对应的目标值。
    g_gimbal_debug.yaw_target = axis.yaw_target;
    // 更新 g_gimbal_debug.pitch_target 对应的目标值。
    g_gimbal_debug.pitch_target = axis.pitch_target;
    // 更新 g_gimbal_debug.yaw_step_hz 对应的本步骤的运行数据。
    g_gimbal_debug.yaw_step_hz = axis.yaw_step_hz;
    // 更新 g_gimbal_debug.pitch_step_hz 对应的本步骤的运行数据。
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
        // 调用 WT901_StartReceive，完成当前步骤的业务处理。
        (void)WT901_StartReceive();
    }
    // 调用 IMU_FollowServiceInit，初始化对应模块或运行状态。
    IMU_FollowServiceInit();
    // 调用 GIMBAL_TaskUpdateDebugStatus，更新并处理对应业务数据。
    GIMBAL_TaskUpdateDebugStatus();
}

/**
 * @brief 在 1ms 节拍到达时处理姿态数据与云台控制业务
 */
void GIMBAL_TaskProcess(void)
{
    // 调用 BSP_Timer_TakeGimbalTicks，读取当前反馈或状态。
    const uint32_t elapsed_ticks = BSP_Timer_TakeGimbalTicks();

    // 每次调度清空已接收的完整帧，只保留最新姿态供跟随业务使用。
    while (WT901_AnalyzeData())
    {
    }

    // 判断本次是否具有有效的控制节拍。
    if (elapsed_ticks == 0U)
    {
        // 返回本次计算或查询得到的结果。
        return;
    }
    // 先由业务层生成目标，再由控制层根据编码器反馈输出 STEP。
    IMU_FollowServiceProcess(elapsed_ticks);
    // 调用 GIMBAL_AxisUpdate，更新并处理对应业务数据。
    GIMBAL_AxisUpdate(elapsed_ticks);
    // 调用 GIMBAL_TaskUpdateDebugStatus，更新并处理对应业务数据。
    GIMBAL_TaskUpdateDebugStatus();
}
