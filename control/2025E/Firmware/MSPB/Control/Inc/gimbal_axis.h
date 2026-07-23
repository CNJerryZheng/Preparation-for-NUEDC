/**
 * @file        gimbal_axis.h
 * @author      JerryZheng
 * @brief       云台轴控制算法层接口
 * @date        2026-07-21
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 云台闭环遥测数据
 */
typedef struct
{
    int32_t yaw_position;      /**< Yaw编码器累计计数。 */
    int32_t pitch_position;    /**< Pitch编码器累计计数。 */
    float yaw_target;          /**< Yaw目标计数。 */
    float pitch_target;        /**< Pitch目标计数。 */
    float yaw_step_hz;         /**< Yaw有符号STEP频率。 */
    float pitch_step_hz;       /**< Pitch有符号STEP频率。 */
    bool vision_valid;         /**< 当前视觉目标是否有效。 */
} GIMBAL_AxisTelemetry_t;

/** @brief 初始化云台两轴编码器位置闭环 */
void GIMBAL_AxisInit(void);

/**
 * @brief 根据树莓派视觉结果更新两轴目标
 * @param valid true表示找到目标，false表示停止追踪并保持当前位置
 * @param laser_x 激光目标图像X坐标
 * @param laser_y 激光目标图像Y坐标
 */
void GIMBAL_AxisSetVisionTarget(
    bool valid, int16_t laser_x, int16_t laser_y);

/**
 * @brief 根据当前编码器、限位和目标值更新云台闭环
 * @param elapsed_ms 距离上次控制更新实际经过的毫秒数
 */
void GIMBAL_AxisUpdate(uint32_t elapsed_ms);

/**
 * @brief 读取云台闭环遥测状态
 * @param telemetry 遥测输出地址
 */
void GIMBAL_AxisGetTelemetry(GIMBAL_AxisTelemetry_t *telemetry);
