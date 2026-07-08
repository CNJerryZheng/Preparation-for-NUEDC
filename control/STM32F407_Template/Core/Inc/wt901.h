/**
 * @file        wt901.h
 * @brief       WT901通信驱动
 * @warning     注意数组越界风险
 * @author      Misybon
 * @date        2026-07-07
*/

#pragma once

#include "usart.h"

/* <---------------WT901 连接的主控外设---------------> */
#define WT901_UART huart1 // 串口
#define WT901_DMA hdma_usart1_rx // DMA 通道
#define WT901_DMA_HT_FLAG DMA_FLAG_HTIF1_5 // DMA 半传输中断标志

/* <--------------------寄存器相关--------------------> */
/**
 * @brief WT901 通信协议中的寄存器对应的字节
 */
typedef enum
{
    WT901_REG_SAVE = 0x00, // 保存/重启/恢复出厂
    WT901_REG_CALSW = 0x01, // 校准模式
    WT901_REG_RSW = 0x02, // 输出内容
    WT901_REG_RRATE = 0x03, // 输出速率
    WT901_REG_BAUD = 0x04, // 串口波特率
    WT901_REG_SLEEP = 0x22, // 休眠
    WT901_REG_READ = 0x27, // 读取寄存器
    WT901_REG_VERSION = 0x2E, // 版本号
    WT901_REG_ACC_X = 0x34, // 加速度 X
    WT901_REG_ACC_Y = 0x35, // 加速度 Y
    WT901_REG_ACC_Z = 0x36, // 加速度 Z
    WT901_REG_GX = 0x37, // 角速度 X
    WT901_REG_GY = 0x38, // 角速度 Y
    WT901_REG_GZ = 0x39, // 角速度 Z
    WT901_REG_ANG_R = 0x3D, // 横滚角
    WT901_REG_ANG_P = 0x3E, // 俯仰角
    WT901_REG_ANG_Y = 0x3F, // 航向角
    WT901_REG_KEY = 0x69, // 解锁
} WT901_RegTypedef;

/* <-------------------通信协议相关-------------------> */
// 宏定义
#define WT901_BUF_SIZE 20 // 缓冲区长度
#define WT901_HEADER_1 0xFF // 帧头 1
#define WT901_HEADER_2 0xAA // 帧头 2

// 类型定义
/**
 * @brief WT901 通信协议中的数据类型对应字节
 */
typedef enum
{
    WT901_DATA_TIME = 0x50, // 时间
    WT901_DATA_ACCEL = 0x51, // 加速度
    WT901_DATA_PALST = 0x52, // 角速度
    WT901_DATA_ANGEL = 0x54, // 角度
    WT901_DATA_READ = 0x5F, // 读取
} WT901_DataTypedef;

/* <---------------------变量相关---------------------> */
// 变量结构体
/**
 * @brief 加速度
 */
struct WT901_Accel
{
    int16_t x;
    int16_t y;
    int16_t z;
};

/**
 * @brief 角速度
 */
struct WT901_Palst
{
    int16_t x;
    int16_t y;
    int16_t z;
};

/**
 * @brief 角度
 */
struct WT901_Angle
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
};

// 外部变量声明
extern const uint8_t WT901_CMD_UNLOCK[5];
extern const uint8_t WT901_CMD_SAVE[5];
extern uint8_t g_wt901_buf[WT901_BUF_SIZE];

/* <---------------------函数相关---------------------> */
/**
 * @brief 向 WT901 发送解锁命令
 */
__STATIC_INLINE void WT901_Unlock(void)
{
    HAL_UART_Transmit_DMA(&WT901_UART, WT901_CMD_UNLOCK, sizeof(WT901_CMD_UNLOCK));
}

/**
 * @brief 向 WT901 发送保存命令
 */
__STATIC_INLINE void WT901_Save(void)
{
    HAL_UART_Transmit_DMA(&WT901_UART, WT901_CMD_SAVE, sizeof(WT901_CMD_SAVE));
}

/**
 * @brief 开启对 WT901 的数据接收
 */
__STATIC_INLINE void WT901_StartReceive(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&WT901_UART, g_wt901_buf, WT901_BUF_SIZE);
}
