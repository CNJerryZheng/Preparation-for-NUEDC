/**
 * @file        bsp_gimbal_config.h
 * @author      JerryZheng
 * @brief       云台步进驱动与编码器硬件极性配置
 * @date        2026-07-23
 */

#pragma once

/** @brief D36A 的 EN 为高电平使能。 */
#define BSP_GIMBAL_ENABLE_ACTIVE_HIGH (1U)

/** @brief Yaw 正方向对应的 DIR 电平，方向相反时改为 0U。 */
#define BSP_GIMBAL_YAW_POSITIVE_DIR_HIGH (1U)

/** @brief Pitch 正方向对应的 DIR 电平，方向相反时改为 0U。 */
#define BSP_GIMBAL_PITCH_POSITIVE_DIR_HIGH (1U)

/** @brief Yaw 编码器方向修正，正向计数为负时改为 -1。 */
#define BSP_GIMBAL_YAW_ENCODER_SIGN (1)

/** @brief Pitch 编码器方向修正，正向计数为负时改为 -1。 */
#define BSP_GIMBAL_PITCH_ENCODER_SIGN (1)
