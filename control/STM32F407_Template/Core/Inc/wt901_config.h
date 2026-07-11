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

/* <---------------WT901 连接的主控外设---------------> */
#define WT901_UART huart1 // 串口
#define WT901_DMA hdma_usart1_rx // DMA 通道
#define WT901_DMA_HT_FLAG DMA_FLAG_HTIF1_5 // DMA 半传输中断标志

/* <-------------------通信协议相关-------------------> */
// 宏定义
#define WT901_CIR_SIZE 256 // 环形缓冲区长度
#define WT901_HEADER_1 0xFF // 帧头 1
#define WT901_HEADER_2 0xAA // 帧头 2
#define WT901_BUF_SIZE 128 // 串口缓冲区长度
