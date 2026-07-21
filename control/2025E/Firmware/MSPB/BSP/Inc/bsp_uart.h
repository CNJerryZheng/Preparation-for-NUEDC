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
 * @brief 处理 GPIOA/GPIOB 共用中断组中的限位输入中断
 */
void GROUP1_IRQHandler(void);
