/**
 * @file        mt6816.h
 * @author      JerryZheng
 * @brief       MT6816 A/B、PWM、Z 六线融合设备驱动接口
 * @date        2026-07-23
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief MT6816 所属云台轴
 */
typedef enum
{
    MT6816_AXIS_YAW = 0, /**< Yaw 编码器。 */
    MT6816_AXIS_PITCH    /**< Pitch 编码器。 */
} MT6816_Axis_t;

/**
 * @brief MT6816 运行状态
 */
typedef struct
{
    int32_t multi_turn_count;   /**< A/B 融合后的多圈位置计数。 */
    uint16_t pwm_angle_code;    /**< PWM 还原的 12 位绝对角度码。 */
    uint32_t pwm_high_ticks;    /**< PWM 高电平捕获计数。 */
    uint32_t pwm_period_ticks;  /**< PWM 完整周期捕获计数。 */
    uint32_t pwm_frame_count;   /**< 已处理的 PWM 捕获帧序号。 */
    uint32_t index_count;       /**< 已接收的 Z 脉冲数量。 */
    bool pwm_valid;             /**< PWM 周期和脉宽是否有效。 */
    bool synchronized;          /**< 是否已经完成绝对位置对齐。 */
} MT6816_Status_t;

/** @brief Yaw MT6816 的 Live Watch 实时状态。 */
extern volatile MT6816_Status_t g_mt6816_yaw_status;

/** @brief Pitch MT6816 的 Live Watch 实时状态。 */
extern volatile MT6816_Status_t g_mt6816_pitch_status;

/**
 * @brief 初始化两只 MT6816 的软件融合状态
 * @note BSP 定时器和 GPIO 必须先完成初始化。
 */
void MT6816_Init(void);

/**
 * @brief 更新 PWM 绝对角度、A/B 多圈累计和 Z 相校准
 * @note 由 1ms 云台任务周期调用，不在中断中执行协议换算。
 */
void MT6816_Update(void);

/**
 * @brief 读取指定轴融合后的多圈位置
 * @param axis 编码器轴
 * @return int32_t 相对机械零点的四倍频累计计数
 */
int32_t MT6816_GetMultiTurnCount(MT6816_Axis_t axis);

/**
 * @brief 将当前位置设置为用户软件零点
 * @param axis 编码器轴
 * @note 只改变用户坐标偏移，不破坏 PWM 和 Z 建立的传感器坐标。
 */
void MT6816_ResetUserPosition(MT6816_Axis_t axis);

/**
 * @brief 判断指定轴是否已完成 PWM 绝对位置同步
 * @param axis 编码器轴
 * @return true 已完成同步
 * @return false 尚未收到有效 PWM
 */
bool MT6816_IsSynchronized(MT6816_Axis_t axis);

/**
 * @brief 读取指定轴 MT6816 的完整状态
 * @param axis 编码器轴
 * @param status 用于接收状态的结构体
 */
void MT6816_GetStatus(MT6816_Axis_t axis, MT6816_Status_t *status);
