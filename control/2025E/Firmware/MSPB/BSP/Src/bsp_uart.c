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

#define BSP_UART_PROTOCOL_RX_BUFFER_SIZE (128U)

/** @brief UART2接收MSPA数据的环形缓冲区。 */
static volatile uint8_t s_mspa_rx_buffer[BSP_UART_PROTOCOL_RX_BUFFER_SIZE];
/** @brief UART2接收缓冲区写入位置。 */
static volatile uint16_t s_mspa_rx_head = 0U;
/** @brief UART2接收缓冲区读取位置。 */
static volatile uint16_t s_mspa_rx_tail = 0U;
/** @brief UART3接收树莓派数据的环形缓冲区。 */
static volatile uint8_t s_rpi_rx_buffer[BSP_UART_PROTOCOL_RX_BUFFER_SIZE];
/** @brief UART3接收缓冲区写入位置。 */
static volatile uint16_t s_rpi_rx_head = 0U;
/** @brief UART3接收缓冲区读取位置。 */
static volatile uint16_t s_rpi_rx_tail = 0U;

/**
 * @brief 使能 MSPB 已配置串口的 NVIC 中断
 */
void BSP_UART_Init(void)
{
    NVIC_EnableIRQ(UART0_TO_ESP_INST_INT_IRQN);
    NVIC_EnableIRQ(UART1_TO_WT901_INST_INT_IRQN);
    NVIC_EnableIRQ(UART2_TO_MSPA_INST_INT_IRQN);
    NVIC_EnableIRQ(UART3_TO_RPI_INST_INT_IRQN);
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
 * @brief 通过指定UART阻塞发送一段短协议帧
 * @param uart UART外设寄存器
 * @param data 待发送数据地址
 * @param length 数据长度
 */
static void BSP_UART_Write(
    UART_Regs *uart, const uint8_t *data, uint16_t length)
{
    if (data == 0)
    {
        return;
    }
    for (uint16_t index = 0U; index < length; ++index)
    {
        DL_UART_Main_transmitDataBlocking(uart, data[index]);
    }
}

/** @copydoc BSP_UART_RpiWrite */
void BSP_UART_RpiWrite(const uint8_t *data, uint16_t length)
{
    BSP_UART_Write(UART3_TO_RPI_INST, data, length);
}

/** @copydoc BSP_UART_MspaWrite */
void BSP_UART_MspaWrite(const uint8_t *data, uint16_t length)
{
    BSP_UART_Write(UART2_TO_MSPA_INST, data, length);
}

/**
 * @brief 从环形缓冲区读取一个字节
 * @param buffer 环形缓冲区
 * @param head 当前写入位置
 * @param tail 当前读取位置地址
 * @param data 接收字节输出地址
 * @return true成功读取，false当前为空
 */
static bool BSP_UART_ReadRing(const volatile uint8_t *buffer,
    volatile uint16_t *head, volatile uint16_t *tail, uint8_t *data)
{
    if ((data == 0) || (*head == *tail))
    {
        return false;
    }

    *data = buffer[*tail];
    *tail = (uint16_t)((*tail + 1U) % BSP_UART_PROTOCOL_RX_BUFFER_SIZE);
    return true;
}

/** @copydoc BSP_UART_RpiReadByte */
bool BSP_UART_RpiReadByte(uint8_t *data)
{
    return BSP_UART_ReadRing(
        s_rpi_rx_buffer, &s_rpi_rx_head, &s_rpi_rx_tail, data);
}

/** @copydoc BSP_UART_MspaReadByte */
bool BSP_UART_MspaReadByte(uint8_t *data)
{
    return BSP_UART_ReadRing(
        s_mspa_rx_buffer, &s_mspa_rx_head, &s_mspa_rx_tail, data);
}

/**
 * @brief 将接收字节写入环形缓冲区
 * @param buffer 环形缓冲区
 * @param head 当前写入位置地址
 * @param tail 当前读取位置
 * @param data 接收字节
 */
static void BSP_UART_WriteRing(volatile uint8_t *buffer,
    volatile uint16_t *head, volatile uint16_t *tail, uint8_t data)
{
    const uint16_t next_head = (uint16_t)(
        (*head + 1U) % BSP_UART_PROTOCOL_RX_BUFFER_SIZE);

    if (next_head != *tail)
    {
        buffer[*head] = data;
        *head = next_head;
    }
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
            DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
            DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
        break;
    }
}

/**
 * @brief UART0 中断服务函数
 * @note ESP 通信协议尚未接入时先取走接收字节，避免落入默认中断处理函数。
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
 * @note 中断仅写环形缓冲区，协议解析在主循环完成。
 */
void UART2_IRQHandler(void)
{
    const DL_UART_IIDX cause =
        DL_UART_Main_getPendingInterrupt(UART2_TO_MSPA_INST);

    if (cause == DL_UART_MAIN_IIDX_RX)
    {
        while (!DL_UART_Main_isRXFIFOEmpty(UART2_TO_MSPA_INST))
        {
            BSP_UART_WriteRing(s_mspa_rx_buffer,
                &s_mspa_rx_head, &s_mspa_rx_tail,
                (uint8_t)DL_UART_Main_receiveData(UART2_TO_MSPA_INST));
        }
    }
    else if (cause != DL_UART_MAIN_IIDX_NO_INTERRUPT)
    {
        DL_UART_Main_clearInterruptStatus(UART2_TO_MSPA_INST,
            DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
            DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
            DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
            DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
            DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    }
}

/**
 * @brief UART3 中断服务函数
 * @note 中断仅写环形缓冲区，协议解析在主循环完成。
 */
void UART3_IRQHandler(void)
{
    const DL_UART_IIDX cause =
        DL_UART_Main_getPendingInterrupt(UART3_TO_RPI_INST);

    if (cause == DL_UART_MAIN_IIDX_RX)
    {
        while (!DL_UART_Main_isRXFIFOEmpty(UART3_TO_RPI_INST))
        {
            BSP_UART_WriteRing(s_rpi_rx_buffer,
                &s_rpi_rx_head, &s_rpi_rx_tail,
                (uint8_t)DL_UART_Main_receiveData(UART3_TO_RPI_INST));
        }
    }
    else if (cause != DL_UART_MAIN_IIDX_NO_INTERRUPT)
    {
        DL_UART_Main_clearInterruptStatus(UART3_TO_RPI_INST,
            DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
            DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
            DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
            DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
            DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    }
}
