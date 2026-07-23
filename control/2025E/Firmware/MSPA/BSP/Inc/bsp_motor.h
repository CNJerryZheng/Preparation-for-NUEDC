/**
 * @file        bsp_motor.h
 * @author      JerryZheng
 * @brief       底盘电机硬件抽象层接口
 * @date        2026-07-22
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 电机硬件通道
 */
typedef enum
{
    BSP_MOTOR_PORT_D = 0,
    BSP_MOTOR_PORT_A
} BSP_MotorChannel_t;

/**
 * @brief 初始化电机 PWM、方向引脚和霍尔计数
 * @note 初始化完成后 TB6612 仍处于待机状态，两路 PWM 占空比均为 0。
 */
void BSP_MotorInit(void);

/**
 * @brief 设置 TB6612 驱动器使能状态
 * @param enable true 退出待机，false 进入待机
 */
void BSP_MotorSetDriverEnable(bool enable);

/**
 * @brief 设置指定电机的方向控制引脚
 * @param channel 电机硬件通道
 * @param in1_high IN1 是否输出高电平
 * @param in2_high IN2 是否输出高电平
 */
void BSP_MotorSetDirection(
    BSP_MotorChannel_t channel, bool in1_high, bool in2_high);

/**
 * @brief 设置指定电机的 PWM 占空比
 * @param channel 电机硬件通道
 * @param duty_permille 占空比千分数，范围 0~1000
 */
void BSP_MotorSetDutyPermille(
    BSP_MotorChannel_t channel, uint16_t duty_permille);

/**
 * @brief 读取指定电机的霍尔 x1 累计计数
 * @param channel 电机硬件通道
 * @return int32_t 带方向的累计脉冲数
 */
int32_t BSP_MotorGetHallCount(BSP_MotorChannel_t channel);

/**
 * @brief 清零指定电机的霍尔累计计数
 * @param channel 电机硬件通道
 */
void BSP_MotorResetHallCount(BSP_MotorChannel_t channel);
