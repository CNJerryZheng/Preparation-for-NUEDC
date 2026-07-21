/**
 * @file        bsp_uart.c
 * @author      JerryZheng
 * @brief       MSPB 串口 BSP 实现
 * @date        2026-07-21
 */

#include "bsp_uart.h"
#include "ti_msp_dl_config.h"
#include "wt901.h"
#include "board.h"

/**
 * @brief 使能 MSPB 已配置串口的 NVIC 中断
 */
void BSP_UART_Init(void)
{
    /* GPIOA、GPIOB 的限位中断共用 GROUP1 向量。 */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(UART0_TO_ESP_INST_INT_IRQN);
    NVIC_EnableIRQ(UART1_TO_WT901_INST_INT_IRQN);
    NVIC_EnableIRQ(UART2_TO_MSPA_INST_INT_IRQN);
    NVIC_EnableIRQ(UART3_TO_RPI_INST_INT_IRQN);
}

/**
 * @brief GPIOA/GPIOB 共用中断服务函数
 * @note 限位状态由 1ms 任务持续读取；此处仅清除下降沿挂起位，避免落入默认中断。
 */
void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
    case GPIO_PITCH_FEEDBACK_INT_IIDX:
        DL_GPIO_clearInterruptStatus(GPIOA,
            GPIO_PITCH_FEEDBACK_PITCH_ENC_A_PIN |
            GPIO_PITCH_FEEDBACK_PITCH_LIMIT_U_PIN |
            GPIO_PITCH_FEEDBACK_PITCH_LIMIT_D_PIN);
        break;

    case GPIO_YAW_LIMIT_INT_IIDX:
        DL_GPIO_clearInterruptStatus(GPIOB,
            GPIO_YAW_LIMIT_YAW_LIMIT_L_PIN |
            GPIO_YAW_LIMIT_YAW_LIMIT_R_PIN);
        break;

    default:
        break;
    }
}

bool BSP_UART_WT901_Transmit(const uint8_t *data, uint32_t length)
{
    uint32_t elapsed_us = 0U;

    if (data == 0)
    {
        return false;
    }
    for (uint32_t index = 0U; index < length; index++)
    {
        while (!DL_UART_Main_transmitDataCheck(UART1_TO_WT901_INST, data[index]))
        {
            /* 对应 F407: HAL_UART_Transmit(..., 100)，整帧最多等待 100ms。 */
            if (elapsed_us >= 100000U)
            {
                return false;
            }
            delay_us(1U);
            elapsed_us++;
        }
    }
    return true;
}

/**
 * @brief 接收 WT901 串口数据并转交设备层解析
 */
void UART1_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART1_TO_WT901_INST))
    {
    case DL_UART_MAIN_IIDX_RX:
        /* UART FIFO 最大 16 字节；限制单次 ISR 工作量，避免异常状态长期占用 CPU。 */
        for (uint32_t index = 0U;
             (index < 16U) && !DL_UART_Main_isRXFIFOEmpty(UART1_TO_WT901_INST);
             index++)
        {
            WT901_CirWrite_Data(
                (uint8_t)DL_UART_Main_receiveData(UART1_TO_WT901_INST));
        }
        break;

    default:
        DL_UART_Main_clearInterruptStatus(UART1_TO_WT901_INST,
            DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
            DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
            DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
            DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
        break;
    }
}

/**
 * @brief UART0 中断服务函数
 * @note ESP 通信协议尚未接入时先取走接收字节，避免落入默认中断处理函数。
 */
void UART0_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART0_TO_ESP_INST) == DL_UART_MAIN_IIDX_RX)
    {
        (void)DL_UART_Main_receiveData(UART0_TO_ESP_INST);
    }
}

/**
 * @brief UART2 中断服务函数
 * @note 与 MSPA 的通信协议尚未接入时先取走接收字节，避免落入默认中断处理函数。
 */
void UART2_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART2_TO_MSPA_INST) == DL_UART_MAIN_IIDX_RX)
    {
        (void)DL_UART_Main_receiveData(UART2_TO_MSPA_INST);
    }
}

/**
 * @brief UART3 中断服务函数
 * @note 树莓派通信协议尚未接入时先取走接收字节，避免落入默认中断处理函数。
 */
void UART3_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART3_TO_RPI_INST) == DL_UART_MAIN_IIDX_RX)
    {
        (void)DL_UART_Main_receiveData(UART3_TO_RPI_INST);
    }
}
