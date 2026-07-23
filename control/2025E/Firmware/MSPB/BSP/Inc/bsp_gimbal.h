/**
 * @file        bsp_gimbal.h
 * @author      JerryZheng
 * @brief       云台步进驱动、编码器及限位 BSP 接口
 * @date        2026-07-23
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 云台物理轴
 */
typedef enum
{
    BSP_GIMBAL_AXIS_YAW = 0,
    BSP_GIMBAL_AXIS_PITCH
} BSP_GimbalAxis_t;

/** @brief 初始化两轴步进输出、编码器累计值和限位中断 */
void BSP_GimbalInit(void);

/**
 * @brief 设置指定轴驱动器使能状态
 * @param axis 云台轴
 * @param enable true 使能D36A，false 失能D36A
 */
void BSP_GimbalSetEnabled(BSP_GimbalAxis_t axis, bool enable);

/**
 * @brief 设置指定轴的STEP脉冲方向和频率
 * @param axis 云台轴
 * @param positive true 为软件定义的正方向，false 为负方向
 * @param frequency_hz STEP脉冲频率；0表示停止发脉冲
 */
void BSP_GimbalSetStep(
    BSP_GimbalAxis_t axis, bool positive, uint32_t frequency_hz);

/**
 * @brief 立即停止指定轴的STEP脉冲
 * @param axis 云台轴
 */
void BSP_GimbalStop(BSP_GimbalAxis_t axis);

/**
 * @brief 读取指定轴的编码器累计计数
 * @param axis 云台轴
 * @return int32_t 经过方向修正的四倍频累计计数
 */
int32_t BSP_GimbalGetEncoderCount(BSP_GimbalAxis_t axis);

/**
 * @brief 将指定轴编码器当前位置设为软件零点
 * @param axis 云台轴
 */
void BSP_GimbalResetEncoder(BSP_GimbalAxis_t axis);

/**
 * @brief 读取Yaw左限位
 * @return true 限位已触发，false 未触发
 */
bool BSP_GimbalIsYawLimitLeftActive(void);

/**
 * @brief 读取Yaw右限位
 * @return true 限位已触发，false 未触发
 */
bool BSP_GimbalIsYawLimitRightActive(void);

/**
 * @brief 读取Pitch上限位
 * @return true 限位已触发，false 未触发
 */
bool BSP_GimbalIsPitchLimitUpActive(void);

/**
 * @brief 读取Pitch下限位
 * @return true 限位已触发，false 未触发
 */
bool BSP_GimbalIsPitchLimitDownActive(void);
