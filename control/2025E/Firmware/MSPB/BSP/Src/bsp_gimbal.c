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
/** @brief Yaw PWM 捕获是否已经越过首个同步周期。 */
static volatile bool s_yaw_pwm_synced = false;
/** @brief Pitch PWM 捕获是否已经越过首个同步周期。 */
static volatile bool s_pitch_pwm_synced = false;
/** @brief Yaw PWM 最近一次高电平宽度。 */
static volatile uint32_t s_yaw_pwm_high_ticks = 0U;
/** @brief Yaw PWM 最近一次完整周期。 */
static volatile uint32_t s_yaw_pwm_period_ticks = 0U;
/** @brief Yaw PWM 有效捕获序号。 */
static volatile uint32_t s_yaw_pwm_sequence = 0U;
/** @brief Yaw PWM 当前有效标志。 */
static volatile bool s_yaw_pwm_valid = false;
/** @brief Pitch PWM 最近一次高电平宽度。 */
static volatile uint32_t s_pitch_pwm_high_ticks = 0U;
/** @brief Pitch PWM 最近一次完整周期。 */
static volatile uint32_t s_pitch_pwm_period_ticks = 0U;
/** @brief Pitch PWM 有效捕获序号。 */
static volatile uint32_t s_pitch_pwm_sequence = 0U;
/** @brief Pitch PWM 当前有效标志。 */
static volatile bool s_pitch_pwm_valid = false;
/** @brief Yaw Z 相脉冲累计序号。 */
static volatile uint32_t s_yaw_index_sequence = 0U;
/** @brief Pitch Z 相脉冲累计序号。 */
static volatile uint32_t s_pitch_index_sequence = 0U;
/** @brief Yaw Z 上升沿到来时的 A/B 累计计数。 */
static volatile int32_t s_yaw_index_encoder_count = 0;
/** @brief Pitch Z 上升沿到来时的 A/B 累计计数。 */
static volatile int32_t s_pitch_index_encoder_count = 0;

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
 * @brief 获取 Yaw 经方向修正后的 A/B 累计计数
 * @return int32_t Yaw 四倍频累计计数
 */
static int32_t BSP_GimbalGetYawEncoderCount(void)
{
    BSP_GimbalUpdateYawEncoder();
    return (int32_t)s_yaw_encoder_extended *
        BSP_GIMBAL_YAW_ENCODER_SIGN;
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
    s_yaw_pwm_synced = false;
    s_pitch_pwm_synced = false;
    s_yaw_pwm_valid = false;
    s_pitch_pwm_valid = false;
    s_yaw_pwm_sequence = 0U;
    s_pitch_pwm_sequence = 0U;
    s_yaw_index_sequence = 0U;
    s_pitch_index_sequence = 0U;

    /**
     * @brief 修正 SDK 2.02 对 TIMA 输入通道 2 的输入源配置
     * @note SysConfig 选择 PA7/TIMA0_CCP2 后，通用驱动仍会把 CC2
     *       错误连接到 CCP0，导致周期捕获正常而高电平宽度恒为零。
     *       此处只修正运行时寄存器，不修改自动生成文件。
     */
    DL_TimerA_setCaptureCompareInput(CAPTURE_YAW_PWM_INST,
        DL_TIMER_CC_INPUT_INV_NOINVERT,
        DL_TIMER_CC_IN_SEL_CCPX,
        DL_TIMER_CC_2_INDEX);

    NVIC_EnableIRQ(GPIO_PITCH_FEEDBACK_INT_IRQN);
    NVIC_EnableIRQ(GPIO_YAW_LIMIT_INT_IRQN);
    NVIC_EnableIRQ(CAPTURE_YAW_PWM_INST_INT_IRQN);
    NVIC_EnableIRQ(CAPTURE_PITCH_PWM_INST_INT_IRQN);
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
        return BSP_GimbalGetYawEncoderCount();
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

/** @copydoc BSP_GimbalGetPwmCapture */
bool BSP_GimbalGetPwmCapture(
    BSP_GimbalAxis_t axis, BSP_GimbalPwmCapture_t *capture)
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    if (capture == 0)
    {
        return false;
    }

    do
    {
        if (axis == BSP_GIMBAL_AXIS_YAW)
        {
            sequence_before = s_yaw_pwm_sequence;
            capture->high_ticks = s_yaw_pwm_high_ticks;
            capture->period_ticks = s_yaw_pwm_period_ticks;
            capture->valid = s_yaw_pwm_valid;
            sequence_after = s_yaw_pwm_sequence;
        }
        else
        {
            sequence_before = s_pitch_pwm_sequence;
            capture->high_ticks = s_pitch_pwm_high_ticks;
            capture->period_ticks = s_pitch_pwm_period_ticks;
            capture->valid = s_pitch_pwm_valid;
            sequence_after = s_pitch_pwm_sequence;
        }
    } while (sequence_before != sequence_after);

    capture->sequence = sequence_after;
    return capture->valid;
}

/** @copydoc BSP_GimbalGetIndexEvent */
bool BSP_GimbalGetIndexEvent(BSP_GimbalAxis_t axis,
    uint32_t *sequence, int32_t *encoder_count)
{
    uint32_t current_sequence;
    int32_t current_count;

    if (axis == BSP_GIMBAL_AXIS_YAW)
    {
        current_sequence = s_yaw_index_sequence;
        current_count = s_yaw_index_encoder_count;
    }
    else
    {
        current_sequence = s_pitch_index_sequence;
        current_count = s_pitch_index_encoder_count;
    }

    if (sequence != 0)
    {
        *sequence = current_sequence;
    }
    if (encoder_count != 0)
    {
        *encoder_count = current_count;
    }
    return current_sequence != 0U;
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
            GPIO_PITCH_FEEDBACK_PITCH_LIMIT_D_PIN |
            GPIO_PITCH_FEEDBACK_YAW_ENC_Z_PIN |
            GPIO_PITCH_FEEDBACK_PITCH_ENC_Z_PIN;
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
        if ((pending & GPIO_PITCH_FEEDBACK_YAW_ENC_Z_PIN) != 0U)
        {
            s_yaw_index_encoder_count =
                BSP_GimbalGetYawEncoderCount();
            s_yaw_index_sequence++;
        }
        if ((pending & GPIO_PITCH_FEEDBACK_PITCH_ENC_Z_PIN) != 0U)
        {
            s_pitch_index_encoder_count =
                s_pitch_encoder_count * BSP_GIMBAL_PITCH_ENCODER_SIGN;
            s_pitch_index_sequence++;
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

/**
 * @brief Yaw MT6816 PWM 捕获中断
 * @note TIMA0 的 C2 保存高电平宽度，C3 保存完整周期；首帧仅用于同步。
 */
void CAPTURE_YAW_PWM_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(CAPTURE_YAW_PWM_INST))
    {
    case DL_TIMERA_IIDX_CC3_DN:
        if (s_yaw_pwm_synced)
        {
            const uint32_t load =
                DL_TimerA_getLoadValue(CAPTURE_YAW_PWM_INST);
            const uint32_t high_capture =
                DL_TimerA_getCaptureCompareValue(
                    CAPTURE_YAW_PWM_INST, DL_TIMER_CC_2_INDEX);
            const uint32_t period_capture =
                DL_TimerA_getCaptureCompareValue(
                    CAPTURE_YAW_PWM_INST, DL_TIMER_CC_3_INDEX);

            s_yaw_pwm_high_ticks = load - high_capture;
            s_yaw_pwm_period_ticks = load - period_capture;
            s_yaw_pwm_valid =
                (s_yaw_pwm_period_ticks > s_yaw_pwm_high_ticks) &&
                (s_yaw_pwm_high_ticks > 0U);
            s_yaw_pwm_sequence++;
        }
        else
        {
            s_yaw_pwm_synced = true;
        }
        DL_TimerA_setTimerCount(CAPTURE_YAW_PWM_INST,
            DL_TimerA_getLoadValue(CAPTURE_YAW_PWM_INST));
        break;

    case DL_TIMERA_IIDX_ZERO:
        s_yaw_pwm_synced = false;
        s_yaw_pwm_valid = false;
        s_yaw_pwm_sequence++;
        break;

    default:
        break;
    }
}

/**
 * @brief Pitch MT6816 PWM 捕获中断
 * @note TIMA1 的 C0 保存高电平宽度，C1 保存完整周期；首帧仅用于同步。
 */
void CAPTURE_PITCH_PWM_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(CAPTURE_PITCH_PWM_INST))
    {
    case DL_TIMERA_IIDX_CC1_DN:
        if (s_pitch_pwm_synced)
        {
            const uint32_t load =
                DL_TimerA_getLoadValue(CAPTURE_PITCH_PWM_INST);
            const uint32_t high_capture =
                DL_TimerA_getCaptureCompareValue(
                    CAPTURE_PITCH_PWM_INST, DL_TIMER_CC_0_INDEX);
            const uint32_t period_capture =
                DL_TimerA_getCaptureCompareValue(
                    CAPTURE_PITCH_PWM_INST, DL_TIMER_CC_1_INDEX);

            s_pitch_pwm_high_ticks = load - high_capture;
            s_pitch_pwm_period_ticks = load - period_capture;
            s_pitch_pwm_valid =
                (s_pitch_pwm_period_ticks > s_pitch_pwm_high_ticks) &&
                (s_pitch_pwm_high_ticks > 0U);
            s_pitch_pwm_sequence++;
        }
        else
        {
            s_pitch_pwm_synced = true;
        }
        DL_TimerA_setTimerCount(CAPTURE_PITCH_PWM_INST,
            DL_TimerA_getLoadValue(CAPTURE_PITCH_PWM_INST));
        break;

    case DL_TIMERA_IIDX_ZERO:
        s_pitch_pwm_synced = false;
        s_pitch_pwm_valid = false;
        s_pitch_pwm_sequence++;
        break;

    default:
        break;
    }
}
