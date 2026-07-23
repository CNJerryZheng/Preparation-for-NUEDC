/**
 * @file        wheel_speed_control_config.h
 * @author      JerryZheng
 * @brief       双轮速度闭环参数配置
 * @date        2026-07-22
 */

#pragma once

/** @brief 轮速 PID 控制周期，当前由 TIMG6 的 10ms 节拍驱动。 */
#define WHEEL_SPEED_CONTROL_PERIOD_S (0.01f)

/** @brief 轮速滑动平均窗口长度，4个采样对应40ms。 */
#define WHEEL_SPEED_AVERAGE_WINDOW_SIZE (4U)

/** @brief 左轮霍尔计数方向修正，前进反馈为负时改为 -1.0f。 */
#define WHEEL_SPEED_LEFT_FEEDBACK_SIGN (1.0f)

/** @brief 右轮霍尔计数方向修正，前进反馈为负时改为 -1.0f。 */
#define WHEEL_SPEED_RIGHT_FEEDBACK_SIGN (1.0f)

/** @brief 位置式速度 PID 默认比例系数。 */
#define WHEEL_SPEED_PID_DEFAULT_KP (0.08f)

/** @brief 位置式速度 PID 默认积分系数。 */
#define WHEEL_SPEED_PID_DEFAULT_KI (0.05f)

/** @brief 位置式速度 PID 默认微分系数。 */
#define WHEEL_SPEED_PID_DEFAULT_KD (0.001f)

/** @brief 左轮默认目标速度，单位为等效 x1 霍尔计数/秒。 */
#define WHEEL_SPEED_DEFAULT_LEFT_TARGET_CPS (1500.0f)

/** @brief 右轮默认目标速度，单位为等效 x1 霍尔计数/秒。 */
#define WHEEL_SPEED_DEFAULT_RIGHT_TARGET_CPS (1500.0f)

/**
 * @brief 速度前馈系数，单位为占空比百分数/(霍尔计数/秒)
 * @note 根据实车开环测试结果固化为 0.043。
 */
#define WHEEL_SPEED_FEEDFORWARD_GAIN (0.043f)

/** @brief PID 输出最小有符号占空比百分数。 */
#define WHEEL_SPEED_PID_OUTPUT_MIN (-100.0f)

/** @brief PID 输出最大有符号占空比百分数。 */
#define WHEEL_SPEED_PID_OUTPUT_MAX (100.0f)

/** @brief VOFA+ 命令允许设置的最大目标速度，单位为霍尔计数/秒。 */
#define WHEEL_SPEED_TARGET_LIMIT_CPS (10000.0f)
