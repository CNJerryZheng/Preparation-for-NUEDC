/**
 * @file        bsp_uart.h
 * @author      JerryZheng
 * @brief       MSPB 串口 BSP 接口
 * @date        2026-07-21
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/** @brief 使能 MSPB 应用需要的串口 NVIC 中断 */
void BSP_UART_Init(void);

/**
 * @brief 通过 UART1 发送 WT901 配置命令
 * @param data 待发送字节数组
 * @param length 数据长度
 * @retval bool 是否成功写入发送 FIFO
 */
bool BSP_UART_WT901_Transmit(const uint8_t *data, uint32_t length);

/**
 * @brief 通过UART3向树莓派发送数据
 * @param data 待发送数据地址
 * @param length 数据长度
 */
void BSP_UART_RpiWrite(const uint8_t *data, uint16_t length);

/**
 * @brief 从UART3树莓派接收缓冲区读取一个字节
 * @param data 接收字节输出地址
 * @retval true 成功读取
 * @retval false 当前没有数据
 */
bool BSP_UART_RpiReadByte(uint8_t *data);

/**
 * @brief 通过UART2向MSPA发送数据
 * @param data 待发送数据地址
 * @param length 数据长度
 */
void BSP_UART_MspaWrite(const uint8_t *data, uint16_t length);

/**
 * @brief 从UART2的MSPA接收缓冲区读取一个字节
 * @param data 接收字节输出地址
 * @retval true 成功读取
 * @retval false 当前没有数据
 */
bool BSP_UART_MspaReadByte(uint8_t *data);
