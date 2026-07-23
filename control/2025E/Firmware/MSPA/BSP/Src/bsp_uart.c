/**
 * @file        bsp_uart.c
 * @author      JerryZheng
 * @brief       MSPA 串口 BSP 实现
 * @date        2026-07-21
 */

#include "bsp_uart.h"
#include "ti_msp_dl_config.h"

#define BSP_UART_VOFA_RX_BUFFER_SIZE (128U)

/** @brief VOFA+ UART3 接收环形缓冲区。 */
static volatile uint8_t s_vofa_rx_buffer[BSP_UART_VOFA_RX_BUFFER_SIZE];
/** @brief VOFA+ UART3 接收写入位置。 */
static volatile uint16_t s_vofa_rx_head = 0U;
/** @brief VOFA+ UART3 接收读取位置。 */
static volatile uint16_t s_vofa_rx_tail = 0U;

/**
 * @brief 使能 MSPA 已配置串口的 NVIC 中断
 */
void BSP_UART_Init(void)
{
    /* 外设参数由 SysConfig 配置，BSP 仅使能 NVIC。 */
    NVIC_EnableIRQ(UART2_TO_MSPB_INST_INT_IRQN);
    NVIC_EnableIRQ(UART0_TO_ESP_INST_INT_IRQN);
    NVIC_EnableIRQ(UART3_TO_VOFA_INST_INT_IRQN);
}

/**
 * @brief 通过 UART3 向 VOFA+ 发送一段数据
 * @param data 待发送数据地址
 * @param length 待发送字节数
 */
void BSP_UART_VofaWrite(const uint8_t *data, uint16_t length)
{
    if (data == 0)
    {
        return;
    }

    for (uint16_t index = 0U; index < length; ++index)
    {
        DL_UART_Main_transmitDataBlocking(UART3_TO_VOFA_INST, data[index]);
    }
}

/** @copydoc BSP_UART_MspbWrite */
void BSP_UART_MspbWrite(const uint8_t *data, uint16_t length)
{
    if (data == 0)
    {
        return;
    }

    for (uint16_t index = 0U; index < length; ++index)
    {
        DL_UART_Main_transmitDataBlocking(
            UART2_TO_MSPB_INST, data[index]);
    }
}

/**
 * @brief 从 UART3 接收环形缓冲区读取一个字节
 * @param data 接收字节输出地址
 * @retval true 成功读取一个字节
 * @retval false 当前没有待处理数据
 */
bool BSP_UART_VofaReadByte(uint8_t *data)
{
    if ((data == 0) || (s_vofa_rx_head == s_vofa_rx_tail))
    {
        return false;
    }

    *data = s_vofa_rx_buffer[s_vofa_rx_tail];
    s_vofa_rx_tail = (uint16_t)((s_vofa_rx_tail + 1U) %
        BSP_UART_VOFA_RX_BUFFER_SIZE);
    return true;
}

/**
 * @brief UART0 中断服务函数
 * @note 当前链路只上报小车进度，接收数据暂时丢弃。
 */
void UART0_IRQHandler(void)
{
    const DL_UART_IIDX cause =
        DL_UART_Main_getPendingInterrupt(UART0_TO_ESP_INST);

    if (cause == DL_UART_MAIN_IIDX_RX)
    {
        while (!DL_UART_Main_isRXFIFOEmpty(UART0_TO_ESP_INST))
        {
            (void)DL_UART_Main_receiveData(UART0_TO_ESP_INST);
        }
    }
    else if (cause != DL_UART_MAIN_IIDX_NO_INTERRUPT)
    {
        DL_UART_Main_clearInterruptStatus(UART0_TO_ESP_INST,
            DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
            DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
            DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
            DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
            DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    }
}

/**
 * @brief UART2 中断服务函数
 * @note 通信协议尚未接入时先取走接收字节，避免落入默认中断处理函数。
 */
void UART2_IRQHandler(void)
{
    const DL_UART_IIDX cause =
        DL_UART_Main_getPendingInterrupt(UART2_TO_MSPB_INST);

    if (cause == DL_UART_MAIN_IIDX_RX)
    {
        while (!DL_UART_Main_isRXFIFOEmpty(UART2_TO_MSPB_INST))
        {
            (void)DL_UART_Main_receiveData(UART2_TO_MSPB_INST);
        }
    }
    else if (cause != DL_UART_MAIN_IIDX_NO_INTERRUPT)
    {
        DL_UART_Main_clearInterruptStatus(UART2_TO_MSPB_INST,
            DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
            DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
            DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
            DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
            DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    }
}

/**
 * @brief UART3 VOFA+ 调参串口中断服务函数
 * @note 接收中断只负责写入环形缓冲区，命令解析在主循环完成。
 */
void UART3_IRQHandler(void)
{
    const DL_UART_IIDX cause =
        DL_UART_Main_getPendingInterrupt(UART3_TO_VOFA_INST);

    if (cause == DL_UART_MAIN_IIDX_RX)
    {
        const uint8_t data =
            (uint8_t)DL_UART_Main_receiveData(UART3_TO_VOFA_INST);
        const uint16_t next_head = (uint16_t)((s_vofa_rx_head + 1U) %
            BSP_UART_VOFA_RX_BUFFER_SIZE);

        if (next_head != s_vofa_rx_tail)
        {
            s_vofa_rx_buffer[s_vofa_rx_head] = data;
            s_vofa_rx_head = next_head;
        }
    }
    else if (cause != DL_UART_MAIN_IIDX_NO_INTERRUPT)
    {
        DL_UART_Main_clearInterruptStatus(UART3_TO_VOFA_INST,
            DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
            DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
            DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
            DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
            DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    }
}
