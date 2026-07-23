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

/**
 * @brief MT6816 PWM 捕获原始数据
 * @note 高电平宽度和周期均为对应 TIMA 定时器的计数值，不在 BSP 层解释角度协议。
 */
typedef struct
{
    uint32_t high_ticks;   /**< PWM 高电平宽度计数。 */
    uint32_t period_ticks; /**< PWM 完整周期计数。 */
    uint32_t sequence;     /**< 每收到一帧有效捕获后递增。 */
    bool valid;            /**< 当前是否持续收到合法 PWM 边沿。 */
} BSP_GimbalPwmCapture_t;

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
 * @brief 读取指定轴 MT6816 的 PWM 捕获快照
 * @param axis 云台轴
 * @param capture 用于接收捕获数据的结构体
 * @return true 当前 PWM 捕获有效
 * @return false 尚未收到完整帧、输入断线或已超时
 */
bool BSP_GimbalGetPwmCapture(
    BSP_GimbalAxis_t axis, BSP_GimbalPwmCapture_t *capture);

/**
 * @brief 读取指定轴最近一次 Z 相事件
 * @param axis 云台轴
 * @param sequence 用于接收 Z 脉冲累计序号，可传入空指针
 * @param encoder_count 用于接收 Z 上升沿时的 A/B 累计值，可传入空指针
 * @return true 至少接收到过一次 Z 脉冲
 * @return false 尚未接收到 Z 脉冲
 */
bool BSP_GimbalGetIndexEvent(BSP_GimbalAxis_t axis,
    uint32_t *sequence, int32_t *encoder_count);

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
