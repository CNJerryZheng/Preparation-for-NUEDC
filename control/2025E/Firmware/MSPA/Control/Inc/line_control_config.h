/**
 * @file        line_control_config.h
 * @author      JerryZheng
 * @brief       八路循迹控制参数配置
 * @date        2026-07-22
 */

#pragma once

/** @brief 圆周率常量。 */
#define LINE_CONTROL_PI (3.1415926f)

/** @brief MG513X 电机减速比，当前为 1:28。 */
#define LINE_CONTROL_GEAR_RATIO (28.0f)

/** @brief 车轮直径，65mm 换算为 0.065m。 */
#define LINE_CONTROL_WHEEL_DIAMETER_M (0.065f)

/** @brief 电机轴旋转一圈对应的等效 x1 霍尔计数，当前电机为13线霍尔。 */
#define LINE_CONTROL_HALL_COUNT_PER_MOTOR_REV (13.0f)

/**
 * @brief 将车轮线速度换算为速度环使用的等效 x1 霍尔计数/秒
 * @param speed_mps 车轮线速度，单位为米/秒
 */
#define LINE_CONTROL_MPS_TO_CPS(speed_mps)                              \
    ((speed_mps) * LINE_CONTROL_HALL_COUNT_PER_MOTOR_REV *              \
        LINE_CONTROL_GEAR_RATIO /                                       \
        (LINE_CONTROL_PI * LINE_CONTROL_WHEEL_DIAMETER_M))

/**
 * @brief 将等效 x1 霍尔计数/秒换算为车轮线速度
 * @param speed_cps 速度环计数值，单位为计数/秒
 */
#define LINE_CONTROL_CPS_TO_MPS(speed_cps)                              \
    ((speed_cps) * LINE_CONTROL_PI * LINE_CONTROL_WHEEL_DIAMETER_M /    \
        (LINE_CONTROL_HALL_COUNT_PER_MOTOR_REV *                        \
            LINE_CONTROL_GEAR_RATIO))

/** @brief 正常直线循迹的基础速度，只需修改此处，单位为米/秒。 */
#define LINE_CONTROL_BASE_SPEED_MPS (0.30f)

/** @brief 根据物理参数自动计算的循迹基础速度，单位为等效 x1 霍尔计数/秒。 */
#define LINE_CONTROL_BASE_SPEED_CPS                                    \
    LINE_CONTROL_MPS_TO_CPS(LINE_CONTROL_BASE_SPEED_MPS)

/** @brief 循迹 PD 比例系数，单位为计数/秒/传感器间距。 */
#define LINE_CONTROL_KP_CPS (120.0f)

/** @brief 循迹 PD 微分系数，抑制快速偏移时的过冲。 */
#define LINE_CONTROL_KD_CPS (50.0f)

/** @brief 黑线每偏离一个传感器间距时，基础速度降低值。 */
#define LINE_CONTROL_TURN_SLOWDOWN_CPS (50.0f)

/** @brief 正常循迹时单轮允许的最低前进速度。 */
#define LINE_CONTROL_MIN_FORWARD_SPEED_CPS (300.0f)

/** @brief 循迹时单轮允许的最大绝对速度目标。 */
#define LINE_CONTROL_MAX_SPEED_CPS (2500.0f)

/** @brief 丢线找线时内侧车轮速度，负数表示反转。 */
#define LINE_CONTROL_LOST_INNER_SPEED_CPS (-300.0f)

/** @brief 丢线找线时外侧车轮速度。 */
#define LINE_CONTROL_LOST_OUTER_SPEED_CPS (1000.0f)

/** @brief 每个 10ms 节拍允许的最大速度目标变化量。 */
#define LINE_CONTROL_SPEED_STEP_CPS_PER_10MS (120.0f)

/**
 * @brief X1 是否安装在车头左侧
 * @note 当前 X1 为 PA27、X8 为 PA7；若实物左右相反，将此宏改为 0U。
 */
#define LINE_CONTROL_X1_ON_LEFT (1U)
