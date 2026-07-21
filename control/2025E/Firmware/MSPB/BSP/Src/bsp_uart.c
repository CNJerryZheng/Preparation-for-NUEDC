/**
 * @file        bsp_uart.c
 * @author      JerryZheng
 * @brief       MSPB UART board-support implementation.
 * @date        2026-07-20
 */

#include "bsp_uart.h"
#include "ti_msp_dl_config.h"
#include "wt901.h"

void BSP_UART_Init(void)
{
    NVIC_EnableIRQ(UART0_TO_ESP_INST_INT_IRQN);
    NVIC_EnableIRQ(UART1_TO_WT901_INST_INT_IRQN);
    NVIC_EnableIRQ(UART2_TO_MSPA_INST_INT_IRQN);
    NVIC_EnableIRQ(UART3_TO_RPI_INST_INT_IRQN);
}

/**
 * @brief Receive WT901 bytes from UART1 and hand them to the driver parser.
 */
void UART1_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART1_TO_WT901_INST))
    {
    case DL_UART_MAIN_IIDX_RX:
        WT901_CirWrite_Data((uint8_t)DL_UART_Main_receiveData(UART1_TO_WT901_INST));
        break;

    default:
        break;
    }
}
