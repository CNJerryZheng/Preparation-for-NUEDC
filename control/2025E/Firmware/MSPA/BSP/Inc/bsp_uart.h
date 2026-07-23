/**
 * @file        bsp_uart.h
 * @author      JerryZheng
 * @brief       MSPA 串口 BSP 接口
 * @date        2026-07-21
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 使能 MSPA 应用需要的串口中断
 */
void BSP_UART_Init(void);

/**
 * @brief 通过 UART3 向 VOFA+ 发送一段数据
 * @param data 待发送数据地址
 * @param length 待发送字节数
 * @note 该接口使用阻塞发送，只能在主循环中调用。
 */
void BSP_UART_VofaWrite(const uint8_t *data, uint16_t length);

/**
 * @brief 通过UART2向MSPB发送一段协议数据
 * @param data 待发送数据地址
 * @param length 待发送字节数
 * @note 该接口用于短帧阻塞发送，只能在主循环中调用。
 */
void BSP_UART_MspbWrite(const uint8_t *data, uint16_t length);

/**
 * @brief 从 UART3 接收环形缓冲区读取一个字节
 * @param data 接收字节输出地址
 * @retval true 成功读取一个字节
 * @retval false 当前没有待处理数据
 */
bool BSP_UART_VofaReadByte(uint8_t *data);
