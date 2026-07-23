/**
 * @file        bsp_motor.c
 * @author      JerryZheng
 * @brief       底盘电机硬件抽象层实现
 * @date        2026-07-22
 */

#include "bsp_motor.h"
#include "ti_msp_dl_config.h"

#define BSP_MOTOR_PWM_PERIOD_COUNTS (4000U)
#define BSP_MOTOR_DUTY_MAX_PERMILLE (1000U)

/** @brief 左轮 TIMG8 QEI 四倍频扩展计数。 */
static uint32_t s_left_hall_count_x4 = 0U;
/** @brief 上一次读取的左轮 16 位硬件 QEI 原始值。 */
static uint16_t s_left_qei_last_raw = 0U;
/** @brief 右轮 GPIO 上升沿软件 x1 累计计数。 */
static volatile uint32_t s_right_hall_count = 0U;

/**
 * @brief 将左轮 16 位硬件 QEI 计数扩展为 32 位有符号累计计数
 * @note 使用有符号 16 位差值自动处理 0 与 65535 之间的回绕；两次读取间
 *       的真实位移必须小于 32768 个四倍频计数。
 */
static void BSP_MotorUpdateLeftQei(void)
{
    const uint16_t current_raw =
        (uint16_t)DL_TimerG_getTimerCount(QEI_LEFT_HALL_INST);
    const int16_t delta =
        (int16_t)(uint16_t)(current_raw - s_left_qei_last_raw);

    s_left_hall_count_x4 += (uint32_t)(int32_t)delta;
    s_left_qei_last_raw = current_raw;
}

/**
 * @brief 根据千分比计算边沿对齐 PWM 的比较值
 * @param duty_permille 占空比千分数
 * @return uint32_t TIMA0 捕获比较值
 */
static uint32_t BSP_MotorDutyToCompare(uint16_t duty_permille)
{
    if (duty_permille > BSP_MOTOR_DUTY_MAX_PERMILLE)
    {
        duty_permille = BSP_MOTOR_DUTY_MAX_PERMILLE;
    }

    return BSP_MOTOR_PWM_PERIOD_COUNTS -
        ((BSP_MOTOR_PWM_PERIOD_COUNTS * (uint32_t)duty_permille) /
            BSP_MOTOR_DUTY_MAX_PERMILLE);
}

/**
 * @brief 初始化电机 PWM、方向引脚和霍尔计数
 */
void BSP_MotorInit(void)
{
    BSP_MotorSetDriverEnable(false);
    BSP_MotorSetDirection(BSP_MOTOR_PORT_D, false, false);
    BSP_MotorSetDirection(BSP_MOTOR_PORT_A, false, false);
    BSP_MotorSetDutyPermille(BSP_MOTOR_PORT_D, 0U);
    BSP_MotorSetDutyPermille(BSP_MOTOR_PORT_A, 0U);
    BSP_MotorResetHallCount(BSP_MOTOR_PORT_D);
    BSP_MotorResetHallCount(BSP_MOTOR_PORT_A);

    DL_TimerA_startCounter(PWM_MOTOR_INST);
    NVIC_EnableIRQ(GPIO_HALL_ENCODER_INT_IRQN);
}

/**
 * @brief 设置 TB6612 驱动器使能状态
 * @param enable true 退出待机，false 进入待机
 */
void BSP_MotorSetDriverEnable(bool enable)
{
    if (enable)
    {
        DL_GPIO_setPins(GPIO_MOTOR_CTRL_PORT,
            GPIO_MOTOR_CTRL_TB6612_STBY_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GPIO_MOTOR_CTRL_PORT,
            GPIO_MOTOR_CTRL_TB6612_STBY_PIN);
    }
}

/**
 * @brief 设置指定电机口的方向控制引脚
 * @param channel 电机硬件通道
 * @param in1_high IN1 是否输出高电平
 * @param in2_high IN2 是否输出高电平
 */
void BSP_MotorSetDirection(
    BSP_MotorChannel_t channel, bool in1_high, bool in2_high)
{
    uint32_t in1_pin;
    uint32_t in2_pin;

    if (channel == BSP_MOTOR_PORT_D)
    {
        in1_pin = GPIO_MOTOR_CTRL_LEFT_MOTOR_IN1_PIN;
        in2_pin = GPIO_MOTOR_CTRL_LEFT_MOTOR_IN2_PIN;
    }
    else
    {
        in1_pin = GPIO_MOTOR_CTRL_RIGHT_MOTOR_IN1_PIN;
        in2_pin = GPIO_MOTOR_CTRL_RIGHT_MOTOR_IN2_PIN;
    }

    if (in1_high)
    {
        DL_GPIO_setPins(GPIO_MOTOR_CTRL_PORT, in1_pin);
    }
    else
    {
        DL_GPIO_clearPins(GPIO_MOTOR_CTRL_PORT, in1_pin);
    }

    if (in2_high)
    {
        DL_GPIO_setPins(GPIO_MOTOR_CTRL_PORT, in2_pin);
    }
    else
    {
        DL_GPIO_clearPins(GPIO_MOTOR_CTRL_PORT, in2_pin);
    }
}

/**
 * @brief 设置指定电机口的 PWM 占空比
 * @param channel 电机硬件通道
 * @param duty_permille 占空比千分数，范围 0~1000
 */
void BSP_MotorSetDutyPermille(
    BSP_MotorChannel_t channel, uint16_t duty_permille)
{
    const uint32_t compare = BSP_MotorDutyToCompare(duty_permille);
    const DL_TIMER_CC_INDEX index =
        (channel == BSP_MOTOR_PORT_D) ? GPIO_PWM_MOTOR_C0_IDX :
                                       GPIO_PWM_MOTOR_C1_IDX;

    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compare, index);
}

/**
 * @brief 读取指定电机口的霍尔原生累计计数
 * @param channel 电机硬件通道
 * @return int32_t 带方向的累计计数；左轮为硬件 x4，右轮为软件 x1
 */
int32_t BSP_MotorGetHallCount(BSP_MotorChannel_t channel)
{
    if (channel == BSP_MOTOR_PORT_D)
    {
        BSP_MotorUpdateLeftQei();
        return (int32_t)s_left_hall_count_x4;
    }

    return (int32_t)s_right_hall_count;
}

/**
 * @brief 清零指定电机口的霍尔累计计数
 * @param channel 电机硬件通道
 */
void BSP_MotorResetHallCount(BSP_MotorChannel_t channel)
{
    if (channel == BSP_MOTOR_PORT_D)
    {
        s_left_hall_count_x4 = 0U;
        s_left_qei_last_raw =
            (uint16_t)DL_TimerG_getTimerCount(QEI_LEFT_HALL_INST);
    }
    else
    {
        s_right_hall_count = 0U;
    }
}

/**
 * @brief GPIOB 组中断服务函数
 * @note 左轮 PB6/PB7 已由 TIMG8 硬件 QEI 四倍频解码；这里只处理右轮
 *       PB8 A 相上升沿，并读取 PB9 B 相实现软件 x1 解码。
 */
void GROUP1_IRQHandler(void)
{
    const uint32_t pending =
        DL_GPIO_getEnabledInterruptStatus(GPIO_HALL_ENCODER_PORT,
            GPIO_HALL_ENCODER_RIGHT_HALL_A_PIN);

    if ((pending & GPIO_HALL_ENCODER_RIGHT_HALL_A_PIN) != 0U)
    {
        if ((DL_GPIO_readPins(GPIO_HALL_ENCODER_PORT,
                 GPIO_HALL_ENCODER_RIGHT_HALL_B_PIN) &
                GPIO_HALL_ENCODER_RIGHT_HALL_B_PIN) != 0U)
        {
            ++s_right_hall_count;
        }
        else
        {
            --s_right_hall_count;
        }
    }

    DL_GPIO_clearInterruptStatus(GPIO_HALL_ENCODER_PORT, pending);
}
