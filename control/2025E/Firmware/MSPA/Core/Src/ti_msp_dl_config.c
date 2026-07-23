/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_MOTORBackup;
DL_TimerG_backupConfig gQEI_LEFT_HALLBackup;
DL_TimerG_backupConfig gTIMER_CHASSIS_10MSBackup;
DL_UART_Main_backupConfig gUART3_TO_VOFABackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_MOTOR_init();
    SYSCFG_DL_QEI_LEFT_HALL_init();
    SYSCFG_DL_TIMER_CHASSIS_10MS_init();
    SYSCFG_DL_UART2_TO_MSPB_init();
    SYSCFG_DL_UART0_TO_ESP_init();
    SYSCFG_DL_UART3_TO_VOFA_init();
    /* Ensure backup structures have no valid state */
	gPWM_MOTORBackup.backupRdy 	= false;
	gQEI_LEFT_HALLBackup.backupRdy 	= false;
	gTIMER_CHASSIS_10MSBackup.backupRdy 	= false;
	gUART3_TO_VOFABackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_MOTOR_INST, &gPWM_MOTORBackup);
	retStatus &= DL_TimerG_saveConfiguration(QEI_LEFT_HALL_INST, &gQEI_LEFT_HALLBackup);
	retStatus &= DL_TimerG_saveConfiguration(TIMER_CHASSIS_10MS_INST, &gTIMER_CHASSIS_10MSBackup);
	retStatus &= DL_UART_Main_saveConfiguration(UART3_TO_VOFA_INST, &gUART3_TO_VOFABackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_MOTOR_INST, &gPWM_MOTORBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(QEI_LEFT_HALL_INST, &gQEI_LEFT_HALLBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(TIMER_CHASSIS_10MS_INST, &gTIMER_CHASSIS_10MSBackup, false);
	retStatus &= DL_UART_Main_restoreConfiguration(UART3_TO_VOFA_INST, &gUART3_TO_VOFABackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_MOTOR_INST);
    DL_TimerG_reset(QEI_LEFT_HALL_INST);
    DL_TimerG_reset(TIMER_CHASSIS_10MS_INST);
    DL_UART_Main_reset(UART2_TO_MSPB_INST);
    DL_UART_Main_reset(UART0_TO_ESP_INST);
    DL_UART_Main_reset(UART3_TO_VOFA_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_MOTOR_INST);
    DL_TimerG_enablePower(QEI_LEFT_HALL_INST);
    DL_TimerG_enablePower(TIMER_CHASSIS_10MS_INST);
    DL_UART_Main_enablePower(UART2_TO_MSPB_INST);
    DL_UART_Main_enablePower(UART0_TO_ESP_INST);
    DL_UART_Main_enablePower(UART3_TO_VOFA_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXOUT_IOMUX);

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_C0_IOMUX,GPIO_PWM_MOTOR_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_C0_PORT, GPIO_PWM_MOTOR_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_MOTOR_C1_IOMUX,GPIO_PWM_MOTOR_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_MOTOR_C1_PORT, GPIO_PWM_MOTOR_C1_PIN);

    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_LEFT_HALL_PHA_IOMUX,GPIO_QEI_LEFT_HALL_PHA_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_LEFT_HALL_PHB_IOMUX,GPIO_QEI_LEFT_HALL_PHB_IOMUX_FUNC);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART2_TO_MSPB_IOMUX_TX, GPIO_UART2_TO_MSPB_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART2_TO_MSPB_IOMUX_RX, GPIO_UART2_TO_MSPB_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART0_TO_ESP_IOMUX_TX, GPIO_UART0_TO_ESP_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART0_TO_ESP_IOMUX_RX, GPIO_UART0_TO_ESP_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART3_TO_VOFA_IOMUX_TX, GPIO_UART3_TO_VOFA_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART3_TO_VOFA_IOMUX_RX, GPIO_UART3_TO_VOFA_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(GPIO_MOTOR_CTRL_TB6612_STBY_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_MOTOR_CTRL_LEFT_MOTOR_IN1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_MOTOR_CTRL_LEFT_MOTOR_IN2_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_MOTOR_CTRL_RIGHT_MOTOR_IN1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_MOTOR_CTRL_RIGHT_MOTOR_IN2_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_MOTOR_CTRL_BUZZER_N_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_HALL_ENCODER_RIGHT_HALL_A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_HALL_ENCODER_RIGHT_HALL_B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_LINE_SENSOR_LINE_1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_LINE_SENSOR_LINE_2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_LINE_SENSOR_LINE_3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_LINE_SENSOR_LINE_4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_LINE_SENSOR_LINE_5_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_LINE_SENSOR_LINE_6_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_LINE_SENSOR_LINE_7_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_LINE_SENSOR_LINE_8_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_USER_INPUT_USER_KEY_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, GPIO_MOTOR_CTRL_TB6612_STBY_PIN |
		GPIO_MOTOR_CTRL_LEFT_MOTOR_IN1_PIN |
		GPIO_MOTOR_CTRL_LEFT_MOTOR_IN2_PIN |
		GPIO_MOTOR_CTRL_RIGHT_MOTOR_IN1_PIN |
		GPIO_MOTOR_CTRL_RIGHT_MOTOR_IN2_PIN);
    DL_GPIO_setPins(GPIOA, GPIO_MOTOR_CTRL_BUZZER_N_PIN);
    DL_GPIO_enableOutput(GPIOA, GPIO_MOTOR_CTRL_TB6612_STBY_PIN |
		GPIO_MOTOR_CTRL_LEFT_MOTOR_IN1_PIN |
		GPIO_MOTOR_CTRL_LEFT_MOTOR_IN2_PIN |
		GPIO_MOTOR_CTRL_RIGHT_MOTOR_IN1_PIN |
		GPIO_MOTOR_CTRL_RIGHT_MOTOR_IN2_PIN |
		GPIO_MOTOR_CTRL_BUZZER_N_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_8_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOB, GPIO_HALL_ENCODER_RIGHT_HALL_A_PIN);
    DL_GPIO_enableInterrupt(GPIOB, GPIO_HALL_ENCODER_RIGHT_HALL_A_PIN);

}


static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
	.rDivClk2x              = 3,
	.rDivClk1               = 0,
	.rDivClk0               = 0,
	.enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_ENABLE,
	.enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
	.enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_DISABLE,
	.sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK2X,
	.sysPLLRef              = DL_SYSCTL_SYSPLL_REF_HFCLK,
	.qDiv                   = 3,
	.pDiv                   = DL_SYSCTL_SYSPLL_PDIV_1
};
static const DL_SYSCTL_LFCLKConfig gLFCLKConfig = {
    .lowCap   = false,
    .monitor  = false,
    .xt1Drive = DL_SYSCTL_LFXT_DRIVE_STRENGTH_HIGHEST,
};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setHFCLKSourceHFXTParams(DL_SYSCTL_HFXT_RANGE_32_48_MHZ,100, true);
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
    /* INT_GROUP1 Priority */
    NVIC_SetPriority(GPIOB_INT_IRQn, 1);

}


/*
 * Timer clock configuration to be sourced by  / 1 (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   80000000 Hz = 80000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_MOTORClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gPWM_MOTORConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 4000,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_MOTOR_init(void) {

    DL_TimerA_setClockConfig(
        PWM_MOTOR_INST, (DL_TimerA_ClockConfig *) &gPWM_MOTORClockConfig);

    DL_TimerA_initPWMMode(
        PWM_MOTOR_INST, (DL_TimerA_PWMConfig *) &gPWM_MOTORConfig);

    DL_TimerA_setCaptureCompareOutCtl(PWM_MOTOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_MOTOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, 4000, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_MOTOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_MOTOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, 4000, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(PWM_MOTOR_INST);


    
    DL_TimerA_setCCPDirection(PWM_MOTOR_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}


static const DL_TimerG_ClockConfig gQEI_LEFT_HALLClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};


SYSCONFIG_WEAK void SYSCFG_DL_QEI_LEFT_HALL_init(void) {

    DL_TimerG_setClockConfig(
        QEI_LEFT_HALL_INST, (DL_TimerG_ClockConfig *) &gQEI_LEFT_HALLClockConfig);

    DL_TimerG_configQEI(QEI_LEFT_HALL_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(QEI_LEFT_HALL_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(QEI_LEFT_HALL_INST, 65535);
    DL_TimerG_enableClock(QEI_LEFT_HALL_INST);
    DL_TimerG_startCounter(QEI_LEFT_HALL_INST);
}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   5000000 Hz = 80000000 Hz / (1 * (15 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_CHASSIS_10MSClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 15U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_CHASSIS_10MS_INST_LOAD_VALUE = (10 ms * 5000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_CHASSIS_10MSTimerConfig = {
    .period     = TIMER_CHASSIS_10MS_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_CHASSIS_10MS_init(void) {

    DL_TimerG_setClockConfig(TIMER_CHASSIS_10MS_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_CHASSIS_10MSClockConfig);

    DL_TimerG_initTimerMode(TIMER_CHASSIS_10MS_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_CHASSIS_10MSTimerConfig);
    DL_TimerG_enableInterrupt(TIMER_CHASSIS_10MS_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
	NVIC_SetPriority(TIMER_CHASSIS_10MS_INST_INT_IRQN, 1);
    DL_TimerG_enableClock(TIMER_CHASSIS_10MS_INST);





}



static const DL_UART_Main_ClockConfig gUART2_TO_MSPBClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART2_TO_MSPBConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART2_TO_MSPB_init(void)
{
    DL_UART_Main_setClockConfig(UART2_TO_MSPB_INST, (DL_UART_Main_ClockConfig *) &gUART2_TO_MSPBClockConfig);

    DL_UART_Main_init(UART2_TO_MSPB_INST, (DL_UART_Main_Config *) &gUART2_TO_MSPBConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(UART2_TO_MSPB_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART2_TO_MSPB_INST, UART2_TO_MSPB_IBRD_40_MHZ_115200_BAUD, UART2_TO_MSPB_FBRD_40_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART2_TO_MSPB_INST,
                                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(UART2_TO_MSPB_INST_INT_IRQN, 2);


    DL_UART_Main_enable(UART2_TO_MSPB_INST);
}

static const DL_UART_Main_ClockConfig gUART0_TO_ESPClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART0_TO_ESPConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART0_TO_ESP_init(void)
{
    DL_UART_Main_setClockConfig(UART0_TO_ESP_INST, (DL_UART_Main_ClockConfig *) &gUART0_TO_ESPClockConfig);

    DL_UART_Main_init(UART0_TO_ESP_INST, (DL_UART_Main_Config *) &gUART0_TO_ESPConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(UART0_TO_ESP_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART0_TO_ESP_INST, UART0_TO_ESP_IBRD_40_MHZ_115200_BAUD, UART0_TO_ESP_FBRD_40_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART0_TO_ESP_INST,
                                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(UART0_TO_ESP_INST_INT_IRQN, 2);


    DL_UART_Main_enable(UART0_TO_ESP_INST);
}

static const DL_UART_Main_ClockConfig gUART3_TO_VOFAClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART3_TO_VOFAConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART3_TO_VOFA_init(void)
{
    DL_UART_Main_setClockConfig(UART3_TO_VOFA_INST, (DL_UART_Main_ClockConfig *) &gUART3_TO_VOFAClockConfig);

    DL_UART_Main_init(UART3_TO_VOFA_INST, (DL_UART_Main_Config *) &gUART3_TO_VOFAConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(UART3_TO_VOFA_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART3_TO_VOFA_INST, UART3_TO_VOFA_IBRD_80_MHZ_115200_BAUD, UART3_TO_VOFA_FBRD_80_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART3_TO_VOFA_INST,
                                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(UART3_TO_VOFA_INST_INT_IRQN, 2);


    DL_UART_Main_enable(UART3_TO_VOFA_INST);
}

