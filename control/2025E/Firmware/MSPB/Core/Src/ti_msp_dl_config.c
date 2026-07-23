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

DL_TimerG_backupConfig gPWM_YAW_STEPBackup;
DL_TimerG_backupConfig gQEI_YAW_ENCODERBackup;
DL_TimerA_backupConfig gCAPTURE_YAW_PWMBackup;
DL_TimerA_backupConfig gCAPTURE_PITCH_PWMBackup;
DL_TimerG_backupConfig gTIMER_GIMBAL_1MSBackup;
DL_UART_Main_backupConfig gUART3_TO_RPIBackup;

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
    SYSCFG_DL_PWM_YAW_STEP_init();
    SYSCFG_DL_PWM_PITCH_STEP_init();
    SYSCFG_DL_QEI_YAW_ENCODER_init();
    SYSCFG_DL_CAPTURE_YAW_PWM_init();
    SYSCFG_DL_CAPTURE_PITCH_PWM_init();
    SYSCFG_DL_TIMER_GIMBAL_1MS_init();
    SYSCFG_DL_UART0_TO_ESP_init();
    SYSCFG_DL_UART1_TO_WT901_init();
    SYSCFG_DL_UART2_TO_MSPA_init();
    SYSCFG_DL_UART3_TO_RPI_init();
    /* Ensure backup structures have no valid state */
	gPWM_YAW_STEPBackup.backupRdy 	= false;
	gQEI_YAW_ENCODERBackup.backupRdy 	= false;
	gCAPTURE_YAW_PWMBackup.backupRdy 	= false;
	gCAPTURE_PITCH_PWMBackup.backupRdy 	= false;
	gTIMER_GIMBAL_1MSBackup.backupRdy 	= false;
	gUART3_TO_RPIBackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_saveConfiguration(PWM_YAW_STEP_INST, &gPWM_YAW_STEPBackup);
	retStatus &= DL_TimerG_saveConfiguration(QEI_YAW_ENCODER_INST, &gQEI_YAW_ENCODERBackup);
	retStatus &= DL_TimerA_saveConfiguration(CAPTURE_YAW_PWM_INST, &gCAPTURE_YAW_PWMBackup);
	retStatus &= DL_TimerA_saveConfiguration(CAPTURE_PITCH_PWM_INST, &gCAPTURE_PITCH_PWMBackup);
	retStatus &= DL_TimerG_saveConfiguration(TIMER_GIMBAL_1MS_INST, &gTIMER_GIMBAL_1MSBackup);
	retStatus &= DL_UART_Main_saveConfiguration(UART3_TO_RPI_INST, &gUART3_TO_RPIBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_restoreConfiguration(PWM_YAW_STEP_INST, &gPWM_YAW_STEPBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(QEI_YAW_ENCODER_INST, &gQEI_YAW_ENCODERBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(CAPTURE_YAW_PWM_INST, &gCAPTURE_YAW_PWMBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(CAPTURE_PITCH_PWM_INST, &gCAPTURE_PITCH_PWMBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(TIMER_GIMBAL_1MS_INST, &gTIMER_GIMBAL_1MSBackup, false);
	retStatus &= DL_UART_Main_restoreConfiguration(UART3_TO_RPI_INST, &gUART3_TO_RPIBackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerG_reset(PWM_YAW_STEP_INST);
    DL_TimerG_reset(PWM_PITCH_STEP_INST);
    DL_TimerG_reset(QEI_YAW_ENCODER_INST);
    DL_TimerA_reset(CAPTURE_YAW_PWM_INST);
    DL_TimerA_reset(CAPTURE_PITCH_PWM_INST);
    DL_TimerG_reset(TIMER_GIMBAL_1MS_INST);
    DL_UART_Main_reset(UART0_TO_ESP_INST);
    DL_UART_Main_reset(UART1_TO_WT901_INST);
    DL_UART_Main_reset(UART2_TO_MSPA_INST);
    DL_UART_Main_reset(UART3_TO_RPI_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerG_enablePower(PWM_YAW_STEP_INST);
    DL_TimerG_enablePower(PWM_PITCH_STEP_INST);
    DL_TimerG_enablePower(QEI_YAW_ENCODER_INST);
    DL_TimerA_enablePower(CAPTURE_YAW_PWM_INST);
    DL_TimerA_enablePower(CAPTURE_PITCH_PWM_INST);
    DL_TimerG_enablePower(TIMER_GIMBAL_1MS_INST);
    DL_UART_Main_enablePower(UART0_TO_ESP_INST);
    DL_UART_Main_enablePower(UART1_TO_WT901_INST);
    DL_UART_Main_enablePower(UART2_TO_MSPA_INST);
    DL_UART_Main_enablePower(UART3_TO_RPI_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXOUT_IOMUX);

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_YAW_STEP_C0_IOMUX,GPIO_PWM_YAW_STEP_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_YAW_STEP_C0_PORT, GPIO_PWM_YAW_STEP_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_PITCH_STEP_C0_IOMUX,GPIO_PWM_PITCH_STEP_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_PITCH_STEP_C0_PORT, GPIO_PWM_PITCH_STEP_C0_PIN);

    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_YAW_ENCODER_PHA_IOMUX,GPIO_QEI_YAW_ENCODER_PHA_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_YAW_ENCODER_PHB_IOMUX,GPIO_QEI_YAW_ENCODER_PHB_IOMUX_FUNC);

    DL_GPIO_initPeripheralInputFunction(GPIO_CAPTURE_YAW_PWM_C2_IOMUX,GPIO_CAPTURE_YAW_PWM_C2_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_CAPTURE_PITCH_PWM_C0_IOMUX,GPIO_CAPTURE_PITCH_PWM_C0_IOMUX_FUNC);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART0_TO_ESP_IOMUX_TX, GPIO_UART0_TO_ESP_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART0_TO_ESP_IOMUX_RX, GPIO_UART0_TO_ESP_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART1_TO_WT901_IOMUX_TX, GPIO_UART1_TO_WT901_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART1_TO_WT901_IOMUX_RX, GPIO_UART1_TO_WT901_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART2_TO_MSPA_IOMUX_TX, GPIO_UART2_TO_MSPA_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART2_TO_MSPA_IOMUX_RX, GPIO_UART2_TO_MSPA_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART3_TO_RPI_IOMUX_TX, GPIO_UART3_TO_RPI_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART3_TO_RPI_IOMUX_RX, GPIO_UART3_TO_RPI_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(GPIO_YAW_CTRL_YAW_DIR_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_YAW_CTRL_YAW_EN_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_YAW_LIMIT_YAW_LIMIT_L_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_YAW_LIMIT_YAW_LIMIT_R_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(GPIO_PITCH_CTRL_PITCH_DIR_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_PITCH_CTRL_PITCH_EN_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_PITCH_CTRL_LASER_EN_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_PITCH_FEEDBACK_PITCH_ENC_A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_PITCH_FEEDBACK_PITCH_ENC_B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInput(GPIO_PITCH_FEEDBACK_PITCH_LIMIT_U_IOMUX);

    DL_GPIO_initDigitalInput(GPIO_PITCH_FEEDBACK_PITCH_LIMIT_D_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_PITCH_FEEDBACK_YAW_ENC_Z_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_PITCH_FEEDBACK_PITCH_ENC_Z_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_USER_INPUT_USER_KEY_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, GPIO_YAW_CTRL_YAW_DIR_PIN |
		GPIO_PITCH_CTRL_PITCH_DIR_PIN |
		GPIO_PITCH_CTRL_PITCH_EN_PIN |
		GPIO_PITCH_CTRL_LASER_EN_PIN);
    DL_GPIO_enableOutput(GPIOA, GPIO_YAW_CTRL_YAW_DIR_PIN |
		GPIO_PITCH_CTRL_PITCH_DIR_PIN |
		GPIO_PITCH_CTRL_PITCH_EN_PIN |
		GPIO_PITCH_CTRL_LASER_EN_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOA, DL_GPIO_PIN_15_EDGE_RISE_FALL |
		DL_GPIO_PIN_0_EDGE_FALL |
		DL_GPIO_PIN_1_EDGE_FALL |
		DL_GPIO_PIN_12_EDGE_RISE |
		DL_GPIO_PIN_13_EDGE_RISE);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_16_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOA, GPIO_PITCH_FEEDBACK_PITCH_ENC_A_PIN |
		GPIO_PITCH_FEEDBACK_PITCH_ENC_B_PIN |
		GPIO_PITCH_FEEDBACK_PITCH_LIMIT_U_PIN |
		GPIO_PITCH_FEEDBACK_PITCH_LIMIT_D_PIN |
		GPIO_PITCH_FEEDBACK_YAW_ENC_Z_PIN |
		GPIO_PITCH_FEEDBACK_PITCH_ENC_Z_PIN);
    DL_GPIO_enableInterrupt(GPIOA, GPIO_PITCH_FEEDBACK_PITCH_ENC_A_PIN |
		GPIO_PITCH_FEEDBACK_PITCH_ENC_B_PIN |
		GPIO_PITCH_FEEDBACK_PITCH_LIMIT_U_PIN |
		GPIO_PITCH_FEEDBACK_PITCH_LIMIT_D_PIN |
		GPIO_PITCH_FEEDBACK_YAW_ENC_Z_PIN |
		GPIO_PITCH_FEEDBACK_PITCH_ENC_Z_PIN);
    DL_GPIO_clearPins(GPIOB, GPIO_YAW_CTRL_YAW_EN_PIN);
    DL_GPIO_enableOutput(GPIOB, GPIO_YAW_CTRL_YAW_EN_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_8_EDGE_FALL |
		DL_GPIO_PIN_9_EDGE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB, GPIO_YAW_LIMIT_YAW_LIMIT_L_PIN |
		GPIO_YAW_LIMIT_YAW_LIMIT_R_PIN);
    DL_GPIO_enableInterrupt(GPIOB, GPIO_YAW_LIMIT_YAW_LIMIT_L_PIN |
		GPIO_YAW_LIMIT_YAW_LIMIT_R_PIN);

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
    NVIC_SetPriority(GPIOA_INT_IRQn, 1);

}


/*
 * Timer clock configuration to be sourced by  / 2 (40000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   40000000 Hz = 40000000 Hz / (2 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gPWM_YAW_STEPClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_2,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gPWM_YAW_STEPConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 40000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_YAW_STEP_init(void) {

    DL_TimerG_setClockConfig(
        PWM_YAW_STEP_INST, (DL_TimerG_ClockConfig *) &gPWM_YAW_STEPClockConfig);

    DL_TimerG_initPWMMode(
        PWM_YAW_STEP_INST, (DL_TimerG_PWMConfig *) &gPWM_YAW_STEPConfig);

    DL_TimerG_setCaptureCompareOutCtl(PWM_YAW_STEP_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_YAW_STEP_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_YAW_STEP_INST, 40000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_enableClock(PWM_YAW_STEP_INST);


    
    DL_TimerG_setCCPDirection(PWM_YAW_STEP_INST , DL_TIMER_CC0_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (40000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   40000000 Hz = 40000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gPWM_PITCH_STEPClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gPWM_PITCH_STEPConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 40000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_PITCH_STEP_init(void) {

    DL_TimerG_setClockConfig(
        PWM_PITCH_STEP_INST, (DL_TimerG_ClockConfig *) &gPWM_PITCH_STEPClockConfig);

    DL_TimerG_initPWMMode(
        PWM_PITCH_STEP_INST, (DL_TimerG_PWMConfig *) &gPWM_PITCH_STEPConfig);

    DL_TimerG_setCaptureCompareOutCtl(PWM_PITCH_STEP_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_PITCH_STEP_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_PITCH_STEP_INST, 40000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_enableClock(PWM_PITCH_STEP_INST);


    
    DL_TimerG_setCCPDirection(PWM_PITCH_STEP_INST , DL_TIMER_CC0_OUTPUT );


}


static const DL_TimerG_ClockConfig gQEI_YAW_ENCODERClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};


SYSCONFIG_WEAK void SYSCFG_DL_QEI_YAW_ENCODER_init(void) {

    DL_TimerG_setClockConfig(
        QEI_YAW_ENCODER_INST, (DL_TimerG_ClockConfig *) &gQEI_YAW_ENCODERClockConfig);

    DL_TimerG_configQEI(QEI_YAW_ENCODER_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(QEI_YAW_ENCODER_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(QEI_YAW_ENCODER_INST, 65535);
    DL_TimerG_enableClock(QEI_YAW_ENCODER_INST);
    DL_TimerG_startCounter(QEI_YAW_ENCODER_INST);
}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   4000000 Hz = 80000000 Hz / (1 * (19 + 1))
 */
static const DL_TimerA_ClockConfig gCAPTURE_YAW_PWMClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 19U
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * CAPTURE_YAW_PWM_INST_LOAD_VALUE = (10 ms * 4000000 Hz) - 1
 */
static const DL_TimerA_CaptureCombinedConfig gCAPTURE_YAW_PWMCaptureConfig = {
    .captureMode    = DL_TIMER_CAPTURE_COMBINED_MODE_PULSE_WIDTH_AND_PERIOD,
    .period         = CAPTURE_YAW_PWM_INST_LOAD_VALUE,
    .startTimer     = DL_TIMER_START,
    .inputChan      = DL_TIMER_INPUT_CHAN_2,
    .inputInvMode   = DL_TIMER_CC_INPUT_INV_NOINVERT,
};

SYSCONFIG_WEAK void SYSCFG_DL_CAPTURE_YAW_PWM_init(void) {

    DL_TimerA_setClockConfig(CAPTURE_YAW_PWM_INST,
        (DL_TimerA_ClockConfig *) &gCAPTURE_YAW_PWMClockConfig);

    DL_TimerA_initCaptureCombinedMode(CAPTURE_YAW_PWM_INST,
        (DL_TimerA_CaptureCombinedConfig *) &gCAPTURE_YAW_PWMCaptureConfig);
    DL_TimerA_enableInterrupt(CAPTURE_YAW_PWM_INST , DL_TIMERA_INTERRUPT_CC3_DN_EVENT |
		DL_TIMERA_INTERRUPT_ZERO_EVENT);

    NVIC_SetPriority(CAPTURE_YAW_PWM_INST_INT_IRQN, 1);
    DL_TimerA_enableClock(CAPTURE_YAW_PWM_INST);

}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   4000000 Hz = 80000000 Hz / (1 * (19 + 1))
 */
static const DL_TimerA_ClockConfig gCAPTURE_PITCH_PWMClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 19U
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * CAPTURE_PITCH_PWM_INST_LOAD_VALUE = (10 ms * 4000000 Hz) - 1
 */
static const DL_TimerA_CaptureCombinedConfig gCAPTURE_PITCH_PWMCaptureConfig = {
    .captureMode    = DL_TIMER_CAPTURE_COMBINED_MODE_PULSE_WIDTH_AND_PERIOD,
    .period         = CAPTURE_PITCH_PWM_INST_LOAD_VALUE,
    .startTimer     = DL_TIMER_START,
    .inputChan      = DL_TIMER_INPUT_CHAN_0,
    .inputInvMode   = DL_TIMER_CC_INPUT_INV_NOINVERT,
};

SYSCONFIG_WEAK void SYSCFG_DL_CAPTURE_PITCH_PWM_init(void) {

    DL_TimerA_setClockConfig(CAPTURE_PITCH_PWM_INST,
        (DL_TimerA_ClockConfig *) &gCAPTURE_PITCH_PWMClockConfig);

    DL_TimerA_initCaptureCombinedMode(CAPTURE_PITCH_PWM_INST,
        (DL_TimerA_CaptureCombinedConfig *) &gCAPTURE_PITCH_PWMCaptureConfig);
    DL_TimerA_enableInterrupt(CAPTURE_PITCH_PWM_INST , DL_TIMERA_INTERRUPT_CC1_DN_EVENT |
		DL_TIMERA_INTERRUPT_ZERO_EVENT);

    NVIC_SetPriority(CAPTURE_PITCH_PWM_INST_INT_IRQN, 1);
    DL_TimerA_enableClock(CAPTURE_PITCH_PWM_INST);

}


/*
 * Timer clock configuration to be sourced by BUSCLK /  (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   40000000 Hz = 80000000 Hz / (1 * (1 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_GIMBAL_1MSClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 1U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_GIMBAL_1MS_INST_LOAD_VALUE = (1 ms * 40000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_GIMBAL_1MSTimerConfig = {
    .period     = TIMER_GIMBAL_1MS_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_GIMBAL_1MS_init(void) {

    DL_TimerG_setClockConfig(TIMER_GIMBAL_1MS_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_GIMBAL_1MSClockConfig);

    DL_TimerG_initTimerMode(TIMER_GIMBAL_1MS_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_GIMBAL_1MSTimerConfig);
    DL_TimerG_enableInterrupt(TIMER_GIMBAL_1MS_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
	NVIC_SetPriority(TIMER_GIMBAL_1MS_INST_INT_IRQN, 1);
    DL_TimerG_enableClock(TIMER_GIMBAL_1MS_INST);





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

static const DL_UART_Main_ClockConfig gUART1_TO_WT901ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART1_TO_WT901Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART1_TO_WT901_init(void)
{
    DL_UART_Main_setClockConfig(UART1_TO_WT901_INST, (DL_UART_Main_ClockConfig *) &gUART1_TO_WT901ClockConfig);

    DL_UART_Main_init(UART1_TO_WT901_INST, (DL_UART_Main_Config *) &gUART1_TO_WT901Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 230400
     *  Actual baud rate: 230547.55
     */
    DL_UART_Main_setOversampling(UART1_TO_WT901_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART1_TO_WT901_INST, UART1_TO_WT901_IBRD_40_MHZ_230400_BAUD, UART1_TO_WT901_FBRD_40_MHZ_230400_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART1_TO_WT901_INST,
                                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(UART1_TO_WT901_INST_INT_IRQN, 2);


    DL_UART_Main_enable(UART1_TO_WT901_INST);
}

static const DL_UART_Main_ClockConfig gUART2_TO_MSPAClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART2_TO_MSPAConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART2_TO_MSPA_init(void)
{
    DL_UART_Main_setClockConfig(UART2_TO_MSPA_INST, (DL_UART_Main_ClockConfig *) &gUART2_TO_MSPAClockConfig);

    DL_UART_Main_init(UART2_TO_MSPA_INST, (DL_UART_Main_Config *) &gUART2_TO_MSPAConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(UART2_TO_MSPA_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART2_TO_MSPA_INST, UART2_TO_MSPA_IBRD_40_MHZ_115200_BAUD, UART2_TO_MSPA_FBRD_40_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART2_TO_MSPA_INST,
                                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(UART2_TO_MSPA_INST_INT_IRQN, 2);


    DL_UART_Main_enable(UART2_TO_MSPA_INST);
}

static const DL_UART_Main_ClockConfig gUART3_TO_RPIClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART3_TO_RPIConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART3_TO_RPI_init(void)
{
    DL_UART_Main_setClockConfig(UART3_TO_RPI_INST, (DL_UART_Main_ClockConfig *) &gUART3_TO_RPIClockConfig);

    DL_UART_Main_init(UART3_TO_RPI_INST, (DL_UART_Main_Config *) &gUART3_TO_RPIConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(UART3_TO_RPI_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART3_TO_RPI_INST, UART3_TO_RPI_IBRD_80_MHZ_115200_BAUD, UART3_TO_RPI_FBRD_80_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART3_TO_RPI_INST,
                                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(UART3_TO_RPI_INST_INT_IRQN, 2);


    DL_UART_Main_enable(UART3_TO_RPI_INST);
}

