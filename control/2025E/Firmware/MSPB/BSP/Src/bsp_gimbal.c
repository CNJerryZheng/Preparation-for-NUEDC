/**
 * @file        bsp_gimbal.c
 * @author      JerryZheng
 * @brief       云台步进驱动、编码器及限位 BSP 实现
 * @date        2026-07-23
 */

#include "bsp_gimbal.h"
#include "bsp_gimbal_config.h"
#include "ti_msp_dl_config.h"

/** @brief Yaw硬件QEI的32位扩展累计值。 */
static uint32_t s_yaw_encoder_extended = 0U;
/** @brief 上一次读取的Yaw硬件QEI 16位值。 */
static uint16_t s_yaw_encoder_last_raw = 0U;
/** @brief Pitch软件四倍频累计值。 */
static volatile int32_t s_pitch_encoder_count = 0;
/** @brief Pitch软件四倍频上一AB状态。 */
static volatile uint8_t s_pitch_encoder_last_state = 0U;

/** @brief 正交编码器合法状态转移查表。 */
static const int8_t s_quadrature_transition[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
};

/**
 * @brief 读取Pitch编码器当前AB状态
 * @return uint8_t bit1为A相，bit0为B相
 */
static uint8_t BSP_GimbalReadPitchEncoderState(void)
{
    const uint32_t pins = DL_GPIO_readPins(GPIO_PITCH_FEEDBACK_PORT,
        GPIO_PITCH_FEEDBACK_PITCH_ENC_A_PIN |
        GPIO_PITCH_FEEDBACK_PITCH_ENC_B_PIN);
    uint8_t state = 0U;

    if ((pins & GPIO_PITCH_FEEDBACK_PITCH_ENC_A_PIN) != 0U)
    {
        state |= 0x02U;
    }
    if ((pins & GPIO_PITCH_FEEDBACK_PITCH_ENC_B_PIN) != 0U)
    {
        state |= 0x01U;
    }
    return state;
}

/**
 * @brief 将Yaw 16位QEI值扩展为32位累计值
 */
static void BSP_GimbalUpdateYawEncoder(void)
{
    const uint16_t current_raw =
        (uint16_t)DL_TimerG_getTimerCount(QEI_YAW_ENCODER_INST);
    const int16_t delta =
        (int16_t)(uint16_t)(current_raw - s_yaw_encoder_last_raw);

    s_yaw_encoder_extended += (uint32_t)(int32_t)delta;
    s_yaw_encoder_last_raw = current_raw;
}

/**
 * @brief 返回指定轴STEP定时器时钟频率
 * @param axis 云台轴
 * @return uint32_t 定时器计数时钟，单位Hz
 */
static uint32_t BSP_GimbalGetStepClock(BSP_GimbalAxis_t axis)
{
    return (axis == BSP_GIMBAL_AXIS_YAW) ?
        PWM_YAW_STEP_INST_CLK_FREQ : PWM_PITCH_STEP_INST_CLK_FREQ;
}

/**
 * @brief 设置指定GPIO为布尔电平
 * @param port GPIO端口
 * @param pin GPIO引脚掩码
 * @param high true输出高电平，false输出低电平
 */
static void BSP_GimbalWritePin(GPIO_Regs *port, uint32_t pin, bool high)
{
    if (high)
    {
        DL_GPIO_setPins(port, pin);
    }
    else
    {
        DL_GPIO_clearPins(port, pin);
    }
}

/** @copydoc BSP_GimbalInit */
void BSP_GimbalInit(void)
{
    BSP_GimbalStop(BSP_GIMBAL_AXIS_YAW);
    BSP_GimbalStop(BSP_GIMBAL_AXIS_PITCH);
    BSP_GimbalSetEnabled(BSP_GIMBAL_AXIS_YAW, false);
    BSP_GimbalSetEnabled(BSP_GIMBAL_AXIS_PITCH, false);

    s_yaw_encoder_extended = 0U;
    s_yaw_encoder_last_raw =
        (uint16_t)DL_TimerG_getTimerCount(QEI_YAW_ENCODER_INST);
    s_pitch_encoder_count = 0;
    s_pitch_encoder_last_state = BSP_GimbalReadPitchEncoderState();

    NVIC_EnableIRQ(GPIO_PITCH_FEEDBACK_INT_IRQN);
    NVIC_EnableIRQ(GPIO_YAW_LIMIT_INT_IRQN);
}

/** @copydoc BSP_GimbalSetEnabled */
void BSP_GimbalSetEnabled(BSP_GimbalAxis_t axis, bool enable)
{
    const bool output_high = (BSP_GIMBAL_ENABLE_ACTIVE_HIGH != 0U) ?
        enable : !enable;

    if (axis == BSP_GIMBAL_AXIS_YAW)
    {
        BSP_GimbalWritePin(GPIO_YAW_CTRL_YAW_EN_PORT,
            GPIO_YAW_CTRL_YAW_EN_PIN, output_high);
    }
    else
    {
        BSP_GimbalWritePin(GPIO_PITCH_CTRL_PORT,
            GPIO_PITCH_CTRL_PITCH_EN_PIN, output_high);
    }
}

/** @copydoc BSP_GimbalSetStep */
void BSP_GimbalSetStep(
    BSP_GimbalAxis_t axis, bool positive, uint32_t frequency_hz)
{
    GPTIMER_Regs *timer;
    DL_TIMER_CC_INDEX cc_index;
    uint32_t period;
    bool direction_high;

    if (frequency_hz == 0U)
    {
        BSP_GimbalStop(axis);
        return;
    }

    if (axis == BSP_GIMBAL_AXIS_YAW)
    {
        timer = PWM_YAW_STEP_INST;
        cc_index = GPIO_PWM_YAW_STEP_C0_IDX;
        direction_high = (BSP_GIMBAL_YAW_POSITIVE_DIR_HIGH != 0U) ?
            positive : !positive;
        BSP_GimbalWritePin(GPIO_YAW_CTRL_YAW_DIR_PORT,
            GPIO_YAW_CTRL_YAW_DIR_PIN, direction_high);
    }
    else
    {
        timer = PWM_PITCH_STEP_INST;
        cc_index = GPIO_PWM_PITCH_STEP_C0_IDX;
        direction_high = (BSP_GIMBAL_PITCH_POSITIVE_DIR_HIGH != 0U) ?
            positive : !positive;
        BSP_GimbalWritePin(GPIO_PITCH_CTRL_PORT,
            GPIO_PITCH_CTRL_PITCH_DIR_PIN, direction_high);
    }

    period = BSP_GimbalGetStepClock(axis) / frequency_hz;
    if (period < 4U)
    {
        period = 4U;
    }

    DL_TimerG_stopCounter(timer);
    DL_TimerG_setLoadValue(timer, period - 1U);
    DL_TimerG_setCaptureCompareValue(timer, period / 2U, cc_index);
    DL_TimerG_setTimerCount(timer, period - 1U);
    DL_TimerG_startCounter(timer);
}

/** @copydoc BSP_GimbalStop */
void BSP_GimbalStop(BSP_GimbalAxis_t axis)
{
    GPTIMER_Regs *timer = (axis == BSP_GIMBAL_AXIS_YAW) ?
        PWM_YAW_STEP_INST : PWM_PITCH_STEP_INST;
    const DL_TIMER_CC_INDEX cc_index = (axis == BSP_GIMBAL_AXIS_YAW) ?
        GPIO_PWM_YAW_STEP_C0_IDX : GPIO_PWM_PITCH_STEP_C0_IDX;

    DL_TimerG_stopCounter(timer);
    DL_TimerG_setCaptureCompareValue(
        timer, DL_TimerG_getLoadValue(timer), cc_index);
}

/** @copydoc BSP_GimbalGetEncoderCount */
int32_t BSP_GimbalGetEncoderCount(BSP_GimbalAxis_t axis)
{
    if (axis == BSP_GIMBAL_AXIS_YAW)
    {
        BSP_GimbalUpdateYawEncoder();
        return (int32_t)s_yaw_encoder_extended *
            BSP_GIMBAL_YAW_ENCODER_SIGN;
    }
    return s_pitch_encoder_count * BSP_GIMBAL_PITCH_ENCODER_SIGN;
}

/** @copydoc BSP_GimbalResetEncoder */
void BSP_GimbalResetEncoder(BSP_GimbalAxis_t axis)
{
    if (axis == BSP_GIMBAL_AXIS_YAW)
    {
        s_yaw_encoder_extended = 0U;
        s_yaw_encoder_last_raw =
            (uint16_t)DL_TimerG_getTimerCount(QEI_YAW_ENCODER_INST);
    }
    else
    {
        s_pitch_encoder_count = 0;
        s_pitch_encoder_last_state = BSP_GimbalReadPitchEncoderState();
    }
}

/** @copydoc BSP_GimbalIsYawLimitLeftActive */
bool BSP_GimbalIsYawLimitLeftActive(void)
{
    return (DL_GPIO_readPins(GPIO_YAW_LIMIT_PORT,
        GPIO_YAW_LIMIT_YAW_LIMIT_L_PIN) &
        GPIO_YAW_LIMIT_YAW_LIMIT_L_PIN) == 0U;
}

/** @copydoc BSP_GimbalIsYawLimitRightActive */
bool BSP_GimbalIsYawLimitRightActive(void)
{
    return (DL_GPIO_readPins(GPIO_YAW_LIMIT_PORT,
        GPIO_YAW_LIMIT_YAW_LIMIT_R_PIN) &
        GPIO_YAW_LIMIT_YAW_LIMIT_R_PIN) == 0U;
}

/** @copydoc BSP_GimbalIsPitchLimitUpActive */
bool BSP_GimbalIsPitchLimitUpActive(void)
{
    return (DL_GPIO_readPins(GPIO_PITCH_FEEDBACK_PORT,
        GPIO_PITCH_FEEDBACK_PITCH_LIMIT_U_PIN) &
        GPIO_PITCH_FEEDBACK_PITCH_LIMIT_U_PIN) == 0U;
}

/** @copydoc BSP_GimbalIsPitchLimitDownActive */
bool BSP_GimbalIsPitchLimitDownActive(void)
{
    return (DL_GPIO_readPins(GPIO_PITCH_FEEDBACK_PORT,
        GPIO_PITCH_FEEDBACK_PITCH_LIMIT_D_PIN) &
        GPIO_PITCH_FEEDBACK_PITCH_LIMIT_D_PIN) == 0U;
}

/**
 * @brief GPIOA/GPIOB共享中断服务函数
 * @note 编码器使用合法正交状态表四倍频解码；任一限位下降沿会立即停止对应轴，
 *       1ms任务仍会持续读取限位电平并禁止继续向限位方向运动。
 */
void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
    case GPIO_PITCH_FEEDBACK_INT_IIDX:
    {
        const uint32_t monitored_pins =
            GPIO_PITCH_FEEDBACK_PITCH_ENC_A_PIN |
            GPIO_PITCH_FEEDBACK_PITCH_ENC_B_PIN |
            GPIO_PITCH_FEEDBACK_PITCH_LIMIT_U_PIN |
            GPIO_PITCH_FEEDBACK_PITCH_LIMIT_D_PIN;
        const uint32_t pending = DL_GPIO_getEnabledInterruptStatus(
            GPIO_PITCH_FEEDBACK_PORT, monitored_pins);

        if ((pending & (GPIO_PITCH_FEEDBACK_PITCH_ENC_A_PIN |
            GPIO_PITCH_FEEDBACK_PITCH_ENC_B_PIN)) != 0U)
        {
            const uint8_t current_state =
                BSP_GimbalReadPitchEncoderState();
            const uint8_t transition = (uint8_t)(
                (s_pitch_encoder_last_state << 2U) | current_state);

            s_pitch_encoder_count += s_quadrature_transition[transition];
            s_pitch_encoder_last_state = current_state;
        }
        if ((pending & (GPIO_PITCH_FEEDBACK_PITCH_LIMIT_U_PIN |
            GPIO_PITCH_FEEDBACK_PITCH_LIMIT_D_PIN)) != 0U)
        {
            BSP_GimbalStop(BSP_GIMBAL_AXIS_PITCH);
        }
        DL_GPIO_clearInterruptStatus(GPIO_PITCH_FEEDBACK_PORT, pending);
        break;
    }

    case GPIO_YAW_LIMIT_INT_IIDX:
    {
        const uint32_t limit_pins =
            GPIO_YAW_LIMIT_YAW_LIMIT_L_PIN |
            GPIO_YAW_LIMIT_YAW_LIMIT_R_PIN;
        const uint32_t pending = DL_GPIO_getEnabledInterruptStatus(
            GPIO_YAW_LIMIT_PORT, limit_pins);

        BSP_GimbalStop(BSP_GIMBAL_AXIS_YAW);
        DL_GPIO_clearInterruptStatus(GPIO_YAW_LIMIT_PORT, pending);
        break;
    }

    default:
        break;
    }
}
