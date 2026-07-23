/**
 * @file        mg513x_config.h
 * @author      JerryZheng
 * @brief       MG513X 霍尔减速电机配置
 * @date        2026-07-22
 */

#pragma once

/** @brief 单个电机轴一圈对应的霍尔线数，当前电机为13线霍尔。 */
#define MG513X_HALL_LINES_PER_MOTOR_REV (13U)

/** @brief 左轮采用 TIMG8 两相硬件 QEI 四倍频解码。 */
#define MG513X_LEFT_HALL_DECODE_MULTIPLIER (4U)

/** @brief 右轮采用 A 相上升沿 GPIO 软件 x1 解码。 */
#define MG513X_RIGHT_HALL_DECODE_MULTIPLIER (1U)

/** @brief 左轮接在底板 Dmotor 口。 */
#define MG513X_LEFT_BSP_CHANNEL BSP_MOTOR_PORT_D

/** @brief 右轮接在底板 Amotor 口。 */
#define MG513X_RIGHT_BSP_CHANNEL BSP_MOTOR_PORT_A

/** @brief 左电机正转时 IN1 的有效电平，若实际方向相反可修改该宏。 */
#define MG513X_LEFT_FORWARD_IN1_HIGH (0U)

/** @brief 右电机正转时 IN1 的有效电平，若实际方向相反可修改该宏。 */
#define MG513X_RIGHT_FORWARD_IN1_HIGH (0U)

/** @brief 电机允许的最大占空比百分数。 */
#define MG513X_MAX_DUTY_PERCENT (100)
