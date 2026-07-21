/**
 * @file        bsp_uart.c
 * @author      JerryZheng
 * @brief       MSPA UART board-support implementation.
 * @date        2026-07-20
 */

#include "bsp_uart.h"
#include "ti_msp_dl_config.h"

void BSP_UART_Init(void)
{
    /* SysConfig owns peripheral setup; the application only enables NVIC. */
    NVIC_EnableIRQ(UART2_TO_MSPB_INST_INT_IRQN);
    NVIC_EnableIRQ(UART3_TO_ESP_INST_INT_IRQN);
}
