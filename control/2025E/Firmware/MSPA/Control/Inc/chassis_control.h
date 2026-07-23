/**
 * @file        chassis_control.h
 * @author      JerryZheng
 * @brief       差速底盘占空比控制接口
 * @date        2026-07-22
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "linetrack.h"

/**
 * @brief 初始化差速底盘控制状态
 * @note 初始化后驱动器失能，左右轮目标占空比均为 0。
 */
void CHASSIS_ControlInit(void);

/**
 * @brief 设置底盘电机驱动总使能
 * @param enable true 允许输出，false 关闭输出
 */
void CHASSIS_SetEnabled(bool enable);

/**
 * @brief 设置八路循迹速度外环是否接管左右轮速度目标
 * @param enable true 启用循迹加轮速闭环，false 停止循迹速度闭环
 */
void CHASSIS_SetLineFollowEnabled(bool enable);

/**
 * @brief 查询八路循迹控制是否已启用
 * @retval bool 当前循迹使能状态
 */
bool CHASSIS_IsLineFollowEnabled(void);

/**
 * @brief 设置双轮速度闭环是否接管电机输出
 * @param enable true 启用速度 PID，false 关闭速度 PID
 * @note 该接口用于固定双轮目标调试；与循迹速度外环互斥。
 */
void CHASSIS_SetSpeedClosedLoopEnabled(bool enable);

/**
 * @brief 查询双轮速度闭环是否启用
 * @retval bool 当前速度闭环使能状态
 */
bool CHASSIS_IsSpeedClosedLoopEnabled(void);

/**
 * @brief 设置双轮速度闭环目标值
 * @param left_cps 左轮目标速度，单位为霍尔计数/秒
 * @param right_cps 右轮目标速度，单位为霍尔计数/秒
 */
void CHASSIS_SetWheelSpeedTarget(float left_cps, float right_cps);

/**
 * @brief 在线设置双轮速度闭环 PID 参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void CHASSIS_SetWheelSpeedPid(float kp, float ki, float kd);

/**
 * @brief 在线设置双轮速度前馈系数
 * @param gain 前馈系数，单位为占空比百分数/(霍尔计数/秒)
 */
void CHASSIS_SetWheelSpeedFeedforward(float gain);

/**
 * @brief 设置左右轮有符号 PWM 占空比
 * @param left_percent 左轮占空比，范围 -100~100
 * @param right_percent 右轮占空比，范围 -100~100
 * @note 正数表示正转，负数表示反转，0 表示滑行停止。
 */
void CHASSIS_SetWheelDutyPercent(
    int16_t left_percent, int16_t right_percent);

/**
 * @brief 停止底盘
 * @param brake true 使用短路制动，false 使用滑行停止
 */
void CHASSIS_Stop(bool brake);

/**
 * @brief 获取当前左右轮目标占空比
 * @param left_percent 左轮目标占空比输出地址，可传入空指针
 * @param right_percent 右轮目标占空比输出地址，可传入空指针
 */
void CHASSIS_GetWheelDutyPercent(
    int16_t *left_percent, int16_t *right_percent);

/**
 * @brief 在 10 ms 控制节拍中更新底盘电机输出
 * @param line 当前循迹结果；循迹启用后由其计算左右轮闭环速度目标
 * @param elapsed_ticks 距离上次更新实际经过的 10ms 节拍数
 */
void CHASSIS_ControlUpdate(
    const LINE_Result_t *line, uint32_t elapsed_ticks);
