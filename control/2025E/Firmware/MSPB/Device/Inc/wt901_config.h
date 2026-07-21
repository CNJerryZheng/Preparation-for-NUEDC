/**
 * @file        wt901_config.h
 * @author      Misybon
                JerryZheng
 * @brief       WT901 配置文件
 * @date        2026-07-11
 */

#pragma once

/* <-------------输出内容（未注释则输出）-------------> */
// #define WT901_GSA_OUT // GPS 定位精度
// #define WT901_QUATER_OUT // 四元数
// #define WT901_VELOCITY_OUT // 地速
// #define WT901_GPS_OUT // 经纬度
// #define WT901_PRESS_OUT // 气压高度
// #define WT901_PORT_OUT // 端口状态
// #define WT901_MAG_OUT // 磁场
#define WT901_ANGLE_OUT // 角度
#define WT901_GYRO_OUT // 角速度
#define WT901_ACC_OUT // 加速度
// #define WT901_TIME_OUT // 时间

/* <---------------------输出速率---------------------> */
// #define WT901_RRATE_0_2HZ // 0.2 赫兹
// #define WT901_RRATE_0_5HZ // 0.5 赫兹
// #define WT901_RRATE_1HZ // 1 赫兹
// #define WT901_RRATE_2HZ // 2 赫兹
// #define WT901_RRATE_5HZ // 5 赫兹
// #define WT901_RRATE_10HZ // 10 赫兹
// #define WT901_RRATE_20HZ // 20 赫兹
// #define WT901_RRATE_50HZ // 50 赫兹
// #define WT901_RRATE_100HZ // 100 赫兹
#define WT901_RRATE_200HZ // 200 赫兹
// #define WT901_RRATE_SINGLE // 单次回传
// #define WT901_RRATE_NO // 不回传

/* <---------------------输出速率---------------------> */
#define WT901_ALG_6 // 六轴算法
// #define WT901_ALG_9 // 九轴算法

/* <---------------WT901 连接的主控外设---------------> */
#define WT901_UART UART1_TO_WT901_INST // 串口
#define WT901_DMA UART1_TO_WT901_INST // MSPM0 使用 UART 中断接收
#define WT901_DMA_HT_FLAG 0U // MSPM0 无 DMA 半传输标志

/* <-------------------通信协议相关-------------------> */
// 宏定义
#define WT901_CIR_SIZE 254 // 环形缓冲区长度
#define WT901_HEADER_1 0xFF // 帧头 1
#define WT901_HEADER_2 0xAA // 帧头 2
#define WT901_BUF_SIZE 128 // 串口缓冲区长度
#define WT901_FRAME_SIZE 11 //数据包长度
#define WT901_FRAME_HEADER 0x55 //数据包包头
#define WT901_FRAME_TAILER 0x0A //数据包包尾

/* MSPM0 UART1 物理波特率；与 Core/MSPB.syscfg 保持一致。 */
#define WT901_UART_BAUD_RATE 230400U
#define WT901_UART_BAUD_REG WT901_BAUD_230400
