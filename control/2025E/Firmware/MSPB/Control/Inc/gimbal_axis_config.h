/**
 * @file        gimbal_axis_config.h
 * @author      JerryZheng
 * @brief       云台编码器闭环与视觉映射参数配置
 * @date        2026-07-23
 */

#pragma once

/** @brief MT6816 AB正交四倍频后的单圈计数。 */
#define GIMBAL_ENCODER_COUNTS_PER_REV (4000.0f)

/** @brief 云台输出轴每旋转一度对应的编码器计数。 */
#define GIMBAL_ENCODER_COUNTS_PER_DEGREE \
    (GIMBAL_ENCODER_COUNTS_PER_REV / 360.0f)

/** @brief Yaw每个图像像素对应的编码器计数，需按相机视场角实测标定。 */
#define GIMBAL_YAW_COUNTS_PER_PIXEL (1.0f)

/** @brief Pitch每个图像像素对应的编码器计数，需按相机视场角实测标定。 */
#define GIMBAL_PITCH_COUNTS_PER_PIXEL (1.0f)

/** @brief 图像X向右时Yaw目标计数符号，方向相反时改为-1.0f。 */
#define GIMBAL_YAW_VISION_SIGN (1.0f)

/** @brief 图像Y向下时Pitch目标计数符号，方向相反时改为-1.0f。 */
#define GIMBAL_PITCH_VISION_SIGN (1.0f)

/** @brief Yaw位置环比例系数，输出单位为STEP脉冲/秒。 */
#define GIMBAL_YAW_POSITION_KP (20.0f)

/** @brief Yaw位置环积分系数。 */
#define GIMBAL_YAW_POSITION_KI (0.0f)

/** @brief Yaw编码器速度反馈阻尼系数。 */
#define GIMBAL_YAW_VELOCITY_KD (0.02f)

/** @brief Pitch位置环比例系数，输出单位为STEP脉冲/秒。 */
#define GIMBAL_PITCH_POSITION_KP (20.0f)

/** @brief Pitch位置环积分系数。 */
#define GIMBAL_PITCH_POSITION_KI (0.0f)

/** @brief Pitch编码器速度反馈阻尼系数。 */
#define GIMBAL_PITCH_VELOCITY_KD (0.02f)

/** @brief 位置误差不超过该计数时停止发STEP脉冲。 */
#define GIMBAL_POSITION_DEADBAND_COUNTS (3.0f)

/** @brief D36A可靠响应的最低STEP频率。 */
#define GIMBAL_MIN_STEP_FREQUENCY_HZ (100.0f)

/** @brief 两轴允许的最高STEP频率。 */
#define GIMBAL_MAX_STEP_FREQUENCY_HZ (8000.0f)

/** @brief 每1ms允许变化的最大STEP频率，用于限制加速度。 */
#define GIMBAL_STEP_FREQUENCY_SLEW_HZ_PER_MS (50.0f)

/** @brief 编码器速度一阶低通滤波时间常数，单位ms。 */
#define GIMBAL_VELOCITY_FILTER_TIME_CONSTANT_MS (20.0f)

/** @brief 位置积分绝对值上限，KI为0时不参与输出。 */
#define GIMBAL_POSITION_INTEGRAL_LIMIT (10000.0f)

/** @brief Yaw相对上电零点允许的最小计数。 */
#define GIMBAL_YAW_MIN_POSITION_COUNTS (-2000.0f)

/** @brief Yaw相对上电零点允许的最大计数。 */
#define GIMBAL_YAW_MAX_POSITION_COUNTS (2000.0f)

/** @brief Pitch相对上电零点允许的最小计数。 */
#define GIMBAL_PITCH_MIN_POSITION_COUNTS (-700.0f)

/** @brief Pitch相对上电零点允许的最大计数。 */
#define GIMBAL_PITCH_MAX_POSITION_COUNTS (700.0f)

/** @brief 超过该时间没有有效视觉帧时停止追踪并保持当前位置。 */
#define GIMBAL_VISION_TIMEOUT_MS (300U)
