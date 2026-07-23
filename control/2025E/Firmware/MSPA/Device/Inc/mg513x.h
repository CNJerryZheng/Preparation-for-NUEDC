/**
 * @file        mg513x.h
 * @author      JerryZheng
 * @brief       MG513X 霍尔减速电机设备层接口
 * @date        2026-07-22
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 底盘电机编号
 */
typedef enum
{
    MG513X_MOTOR_LEFT = 0,
    MG513X_MOTOR_RIGHT
} MG513X_Motor_t;

/**
 * @brief 初始化两路 MG513X 电机
 * @note 初始化后驱动器保持失能，两路电机均停止。
 */
void MG513X_Init(void);

/**
 * @brief 设置 TB6612 驱动器总使能状态
 * @param enable true 允许电机输出，false 关闭电机输出
 */
void MG513X_SetEnabled(bool enable);

/**
 * @brief 按有符号占空比驱动指定电机
 * @param motor 电机编号
 * @param duty_percent 有符号占空比，范围 -100~100；正数正转，负数反转，0 滑行停止
 */
void MG513X_SetDutyPercent(MG513X_Motor_t motor, int16_t duty_percent);

/**
 * @brief 使指定电机短路制动
 * @param motor 电机编号
 */
void MG513X_Brake(MG513X_Motor_t motor);

/**
 * @brief 读取指定电机的霍尔原生累计计数
 * @param motor 电机编号
 * @return int32_t 带方向的累计计数，具体倍频数由配置文件决定
 */
int32_t MG513X_GetHallCount(MG513X_Motor_t motor);

/**
 * @brief 读取指定电机当前使用的霍尔解码倍频数
 * @param motor 电机编号
 * @return uint8_t 软件 x1 返回 1，硬件四倍频 QEI 返回 4
 */
uint8_t MG513X_GetHallDecodeMultiplier(MG513X_Motor_t motor);

/**
 * @brief 清零指定电机的霍尔累计计数
 * @param motor 电机编号
 */
void MG513X_ResetHallCount(MG513X_Motor_t motor);

/**
 * @brief 根据配置的13线霍尔计数计算电机轴累计圈数
 * @param motor 电机编号
 * @return float 电机轴累计旋转圈数
 * @note 该返回值是减速箱输入端电机轴圈数；输出轴圈数还需除以实际减速比。
 */
float MG513X_GetMotorRevolutions(MG513X_Motor_t motor);
