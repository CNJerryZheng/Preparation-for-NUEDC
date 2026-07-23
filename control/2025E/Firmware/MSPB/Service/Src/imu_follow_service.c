/**
 * @file        imu_follow_service.c
 * @author      JerryZheng
 * @brief       WT901 姿态跟随云台业务实现
 * @date        2026-07-23
 */

#include "imu_follow_service.h"
#include "imu_follow_service_config.h"
#include "gimbal_axis.h"
#include "wt901.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief Live Watch：WT901 跟随是否正在工作。 */
volatile uint8_t g_wt901_follow_active = 0U;
/** @brief Live Watch：滤波后的 Yaw 相对角度。 */
volatile float g_wt901_follow_yaw_deg = 0.0f;
/** @brief Live Watch：滤波后的 Pitch 相对角度。 */
volatile float g_wt901_follow_pitch_deg = 0.0f;
/** @brief Live Watch：距离最近 WT901 角度帧的时间。 */
volatile uint32_t g_wt901_follow_age_ms = IMU_FOLLOW_TIMEOUT_MS;

/** @brief 上次处理的 WT901 角度帧序号。 */
static uint32_t s_last_angle_update_count = 0U;
/** @brief 上一帧 WT901 yaw。 */
static float s_last_yaw_deg = 0.0f;
/** @brief 上一帧 WT901 pitch。 */
static float s_last_pitch_deg = 0.0f;
/** @brief 从跟随零点开始累计的 Yaw 解缠角度。 */
static float s_unwrapped_yaw_deg = 0.0f;
/** @brief 从跟随零点开始累计的 Pitch 解缠角度。 */
static float s_unwrapped_pitch_deg = 0.0f;
/** @brief 上一次提交给控制层的 Yaw 目标角度。 */
static float s_published_yaw_deg = 0.0f;
/** @brief 上一次提交给控制层的 Pitch 目标角度。 */
static float s_published_pitch_deg = 0.0f;

/**
 * @brief 计算浮点数绝对值
 * @param value 原始数值
 * @return float 非负绝对值
 */
static float IMU_FollowAbs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 将相邻姿态角差值限制到 -180°～180°
 * @param delta_deg 未处理的角度差
 * @return float 完成跨界解缠的最短角度差
 */
static float IMU_FollowWrapDelta(float delta_deg)
{
    // WT901 航向角跨越正负180°时选择物理上更短的连续转角。
    if (delta_deg > 180.0f)
    {
        delta_deg -= 360.0f;
    }
    else if (delta_deg < -180.0f)
    {
        delta_deg += 360.0f;
    }
    return delta_deg;
}

/**
 * @brief 使用饱和加法累计数据超时时间
 * @param current_ms 当前累计时间
 * @param elapsed_ms 本次经过时间
 * @return uint32_t 不发生无符号溢出的累计结果
 */
static uint32_t IMU_FollowAccumulateTime(
    uint32_t current_ms, uint32_t elapsed_ms)
{
    if (current_ms > (UINT32_MAX - elapsed_ms))
    {
        return UINT32_MAX;
    }
    return current_ms + elapsed_ms;
}

/**
 * @brief 使用当前姿态和云台位置重新建立跟随零点
 * @return true 成功进入跟随状态
 * @return false 云台编码器尚未就绪
 */
static bool IMU_FollowBeginReference(void)
{
    // 编码器未完成绝对位置同步时禁止建立跟随参考点。
    if (!GIMBAL_AxisBeginAngleFollow())
    {
        return false;
    }

    // 当前 WT901 姿态和云台位置共同作为本次跟随的相对零点。
    s_last_yaw_deg = g_wt901_angle.yaw;
    s_last_pitch_deg = g_wt901_angle.pitch;
    s_unwrapped_yaw_deg = 0.0f;
    s_unwrapped_pitch_deg = 0.0f;
    g_wt901_follow_yaw_deg = 0.0f;
    g_wt901_follow_pitch_deg = 0.0f;
    s_published_yaw_deg = 0.0f;
    s_published_pitch_deg = 0.0f;
    g_wt901_follow_age_ms = 0U;
    g_wt901_follow_active = 1U;
    return true;
}

/** @copydoc IMU_FollowServiceInit */
void IMU_FollowServiceInit(void)
{
    s_last_angle_update_count = g_wt901_angle_update_count;
    s_last_yaw_deg = 0.0f;
    s_last_pitch_deg = 0.0f;
    s_unwrapped_yaw_deg = 0.0f;
    s_unwrapped_pitch_deg = 0.0f;
    s_published_yaw_deg = 0.0f;
    s_published_pitch_deg = 0.0f;
    g_wt901_follow_yaw_deg = 0.0f;
    g_wt901_follow_pitch_deg = 0.0f;
    g_wt901_follow_age_ms = IMU_FOLLOW_TIMEOUT_MS;
    g_wt901_follow_active = 0U;
}

/** @copydoc IMU_FollowServiceProcess */
void IMU_FollowServiceProcess(uint32_t elapsed_ms)
{
#if IMU_FOLLOW_ENABLE
    const uint32_t current_update_count = g_wt901_angle_update_count;

    // 没有新的控制节拍时不重复计算同一帧数据。
    if (elapsed_ms == 0U)
    {
        return;
    }

    // 两轴反馈未就绪时只跟踪最新帧号，不允许发布运动目标。
    if (!GIMBAL_AxisIsReady())
    {
        g_wt901_follow_active = 0U;
        s_last_angle_update_count = current_update_count;
        g_wt901_follow_age_ms = 0U;
        return;
    }

    // 帧序号变化表示收到新的角度帧，可以更新相对姿态目标。
    if (current_update_count != s_last_angle_update_count)
    {
        const uint32_t sample_elapsed_ms =
            IMU_FollowAccumulateTime(
                g_wt901_follow_age_ms, elapsed_ms);
        float filter_alpha;
        float yaw_target_deg;
        float pitch_target_deg;

        s_last_angle_update_count = current_update_count;
        // 首个有效角度帧只建立零点，下一帧开始计算相对转角。
        if (g_wt901_follow_active == 0U)
        {
            (void)IMU_FollowBeginReference();
            return;
        }

        // 累计相邻帧最短角度差，使 Yaw 能连续跨越正负180°。
        s_unwrapped_yaw_deg += IMU_FollowWrapDelta(
            g_wt901_angle.yaw - s_last_yaw_deg);
        s_unwrapped_pitch_deg += IMU_FollowWrapDelta(
            g_wt901_angle.pitch - s_last_pitch_deg);
        s_last_yaw_deg = g_wt901_angle.yaw;
        s_last_pitch_deg = g_wt901_angle.pitch;

        // 一阶低通抑制 WT901 静止噪声直接传递到步进电机。
        filter_alpha = (float)sample_elapsed_ms /
            (IMU_FOLLOW_FILTER_TIME_CONSTANT_MS +
             (float)sample_elapsed_ms);
        g_wt901_follow_yaw_deg += filter_alpha *
            (s_unwrapped_yaw_deg - g_wt901_follow_yaw_deg);
        g_wt901_follow_pitch_deg += filter_alpha *
            (s_unwrapped_pitch_deg - g_wt901_follow_pitch_deg);
        g_wt901_follow_age_ms = 0U;

#if IMU_FOLLOW_YAW_ENABLE
        // 比例系数决定跟随倍率，符号参数用于统一传感器与电机正方向。
        yaw_target_deg = g_wt901_follow_yaw_deg *
            IMU_FOLLOW_YAW_SCALE * IMU_FOLLOW_YAW_SIGN;
#else
        yaw_target_deg = 0.0f;
#endif
#if IMU_FOLLOW_PITCH_ENABLE
        // Pitch 与 Yaw 独立配置，便于按机构方向分别校正。
        pitch_target_deg = g_wt901_follow_pitch_deg *
            IMU_FOLLOW_PITCH_SCALE * IMU_FOLLOW_PITCH_SIGN;
#else
        pitch_target_deg = 0.0f;
#endif

        // 只有目标变化超过死区才提交，防止静止时频繁刷新目标。
        if ((IMU_FollowAbs(
                yaw_target_deg - s_published_yaw_deg) >=
                IMU_FOLLOW_TARGET_DEADBAND_DEG) ||
            (IMU_FollowAbs(
                pitch_target_deg - s_published_pitch_deg) >=
                IMU_FOLLOW_TARGET_DEADBAND_DEG))
        {
            s_published_yaw_deg = yaw_target_deg;
            s_published_pitch_deg = pitch_target_deg;
            GIMBAL_AxisSetRelativeAngleTarget(
                yaw_target_deg, pitch_target_deg);
        }
        return;
    }

    // 没有新帧时累计数据年龄，超时后保持当前位置并退出本次跟随。
    g_wt901_follow_age_ms = IMU_FollowAccumulateTime(
        g_wt901_follow_age_ms, elapsed_ms);
    if ((g_wt901_follow_active != 0U) &&
        (g_wt901_follow_age_ms > IMU_FOLLOW_TIMEOUT_MS))
    {
        GIMBAL_AxisHoldCurrentPosition();
        g_wt901_follow_active = 0U;
    }
#else
    (void)elapsed_ms;
#endif
}
