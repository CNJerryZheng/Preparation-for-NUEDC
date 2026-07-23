/**
 * @file        gimbal_device.h
 * @author      JerryZheng
 * @brief       D36A步进驱动器与MT6816编码器设备层接口
 * @date        2026-07-23
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 云台设备轴编号
 */
typedef enum
{
    GIMBAL_DEVICE_AXIS_YAW = 0, /**< 水平Yaw轴。 */
    GIMBAL_DEVICE_AXIS_PITCH    /**< 俯仰Pitch轴。 */
} GIMBAL_DeviceAxis_t;

/**
 * @brief 初始化两轴D36A、MT6816编码器和限位输入
 * @note 初始化结束后两轴驱动器保持失能，STEP输出停止。
 */
void GIMBAL_DeviceInit(void);

/**
 * @brief 设置指定轴D36A使能状态
 * @param axis 云台设备轴
 * @param enable true使能驱动器，false失能驱动器
 */
void GIMBAL_DeviceSetEnabled(
    GIMBAL_DeviceAxis_t axis, bool enable);

/**
 * @brief 设置指定轴运动方向和STEP频率
 * @param axis 云台设备轴
 * @param positive true表示软件正方向，false表示软件负方向
 * @param frequency_hz STEP脉冲频率，0表示停止
 */
void GIMBAL_DeviceSetStep(GIMBAL_DeviceAxis_t axis,
    bool positive, uint32_t frequency_hz);

/**
 * @brief 立即停止指定轴STEP脉冲
 * @param axis 云台设备轴
 */
void GIMBAL_DeviceStop(GIMBAL_DeviceAxis_t axis);

/**
 * @brief 读取指定轴MT6816 AB正交累计计数
 * @param axis 云台设备轴
 * @return int32_t 经过方向修正的四倍频累计计数
 */
int32_t GIMBAL_DeviceGetEncoderCount(GIMBAL_DeviceAxis_t axis);

/**
 * @brief 将指定轴编码器当前位置设为软件零点
 * @param axis 云台设备轴
 */
void GIMBAL_DeviceResetEncoder(GIMBAL_DeviceAxis_t axis);

/**
 * @brief 判断指定运动方向是否被当前限位开关禁止
 * @param axis 云台设备轴
 * @param positive true准备正向运动，false准备负向运动
 * @return true 对应方向限位已触发
 * @return false 对应方向可以运动
 */
bool GIMBAL_DeviceIsDirectionBlocked(
    GIMBAL_DeviceAxis_t axis, bool positive);

