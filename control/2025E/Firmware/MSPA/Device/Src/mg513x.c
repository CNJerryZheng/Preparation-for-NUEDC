/**
 * @file        mg513x.c
 * @author      JerryZheng
 * @brief       MG513X 霍尔减速电机设备层实现
 * @date        2026-07-22
 */

#include "mg513x.h"
#include "mg513x_config.h"
#include "bsp_motor.h"

/**
 * @brief 将设备层电机编号转换为 BSP 电机通道
 * @param motor 电机编号
 * @return BSP_MotorChannel_t BSP 电机硬件通道
 */
static BSP_MotorChannel_t MG513X_ToBspChannel(MG513X_Motor_t motor)
{
    return (motor == MG513X_MOTOR_LEFT) ?
        MG513X_LEFT_BSP_CHANNEL : MG513X_RIGHT_BSP_CHANNEL;
}

/**
 * @brief 取得指定电机正转时 IN1 的有效电平
 * @param motor 电机编号
 * @return bool IN1 正转有效电平
 */
static bool MG513X_GetForwardIn1Level(MG513X_Motor_t motor)
{
    return (motor == MG513X_MOTOR_LEFT) ?
        (MG513X_LEFT_FORWARD_IN1_HIGH != 0U) :
        (MG513X_RIGHT_FORWARD_IN1_HIGH != 0U);
}

/**
 * @brief 初始化两路 MG513X 电机
 */
void MG513X_Init(void)
{
    BSP_MotorInit();
}

/**
 * @brief 设置 TB6612 驱动器总使能状态
 * @param enable true 允许输出，false 关闭输出
 */
void MG513X_SetEnabled(bool enable)
{
    BSP_MotorSetDriverEnable(enable);
}

/**
 * @brief 按有符号占空比驱动指定电机
 * @param motor 电机编号
 * @param duty_percent 有符号占空比，范围 -100~100
 */
void MG513X_SetDutyPercent(MG513X_Motor_t motor, int16_t duty_percent)
{
    const BSP_MotorChannel_t channel = MG513X_ToBspChannel(motor);
    const bool forward_in1 = MG513X_GetForwardIn1Level(motor);
    uint16_t duty_permille;

    if (duty_percent > MG513X_MAX_DUTY_PERCENT)
    {
        duty_percent = MG513X_MAX_DUTY_PERCENT;
    }
    else if (duty_percent < -MG513X_MAX_DUTY_PERCENT)
    {
        duty_percent = -MG513X_MAX_DUTY_PERCENT;
    }

    if (duty_percent == 0)
    {
        BSP_MotorSetDutyPermille(channel, 0U);
        BSP_MotorSetDirection(channel, false, false);
        return;
    }

    if (duty_percent > 0)
    {
        BSP_MotorSetDirection(channel, forward_in1, !forward_in1);
        duty_permille = (uint16_t)duty_percent * 10U;
    }
    else
    {
        BSP_MotorSetDirection(channel, !forward_in1, forward_in1);
        duty_permille = (uint16_t)(-duty_percent) * 10U;
    }

    BSP_MotorSetDutyPermille(channel, duty_permille);
}

/**
 * @brief 使指定电机短路制动
 * @param motor 电机编号
 */
void MG513X_Brake(MG513X_Motor_t motor)
{
    const BSP_MotorChannel_t channel = MG513X_ToBspChannel(motor);

    BSP_MotorSetDutyPermille(channel, 0U);
    BSP_MotorSetDirection(channel, true, true);
}

/**
 * @brief 读取指定电机的霍尔原生累计计数
 * @param motor 电机编号
 * @return int32_t 带方向的累计计数，具体倍频数由配置文件决定
 */
int32_t MG513X_GetHallCount(MG513X_Motor_t motor)
{
    return BSP_MotorGetHallCount(MG513X_ToBspChannel(motor));
}

/**
 * @brief 读取指定电机当前使用的霍尔解码倍频数
 * @param motor 电机编号
 * @return uint8_t 软件 x1 返回 1，硬件四倍频 QEI 返回 4
 */
uint8_t MG513X_GetHallDecodeMultiplier(MG513X_Motor_t motor)
{
    return (motor == MG513X_MOTOR_LEFT) ?
        MG513X_LEFT_HALL_DECODE_MULTIPLIER :
        MG513X_RIGHT_HALL_DECODE_MULTIPLIER;
}

/**
 * @brief 清零指定电机的霍尔累计计数
 * @param motor 电机编号
 */
void MG513X_ResetHallCount(MG513X_Motor_t motor)
{
    BSP_MotorResetHallCount(MG513X_ToBspChannel(motor));
}

/**
 * @brief 根据霍尔计数计算电机轴累计圈数
 * @param motor 电机编号
 * @return float 电机轴累计旋转圈数
 */
float MG513X_GetMotorRevolutions(MG513X_Motor_t motor)
{
    return (float)MG513X_GetHallCount(motor) /
        ((float)MG513X_HALL_LINES_PER_MOTOR_REV *
            (float)MG513X_GetHallDecodeMultiplier(motor));
}
