/**
 * @file        bsp_uart.c
 * @author      JerryZheng
 * @brief       MSPA 串口 BSP 实现
 * @date        2026-07-21
 */

#include "bsp_uart.h"
#include "ti_msp_dl_config.h"

/**
 * @brief 使能 MSPA 已配置串口的 NVIC 中断
 */
void BSP_UART_Init(void)
{
    /* 外设参数由 SysConfig 配置，BSP 仅使能 NVIC。 */
    NVIC_EnableIRQ(UART2_TO_MSPB_INST_INT_IRQN);
    NVIC_EnableIRQ(UART0_TO_ESP_INST_INT_IRQN);
}

/**
 * @brief UART0 中断服务函数
 * @note 通信协议尚未接入时先取走接收字节，避免落入默认中断处理函数。
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
 * @note 通信协议尚未接入时先取走接收字节，避免落入默认中断处理函数。
 */
void UART2_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART2_TO_MSPB_INST) == DL_UART_MAIN_IIDX_RX)
    {
        (void)DL_UART_Main_receiveData(UART2_TO_MSPB_INST);
    }
}
