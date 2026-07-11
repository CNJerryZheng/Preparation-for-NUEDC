/**
 * @file        wt901.h
 * @author      Misybon
                JerryZheng
 * @brief       WT901通信驱动
 * @warning     注意数组越界风险
 * @date        2026-07-11
*/

#pragma once

#include "usart.h"
#include "wt901_config.h"

#ifdef __cplusplus
extern "C" {
#endif

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
} WT901_RegTypeDef;

/**
 * @brief SAVE 寄存器可选的写入值
 */
typedef enum
{
    WT901_SAVE_SAVE = 0x0000, // 保存值
    WT901_SAVE_RESTART = 0x00FF, // 重启值
    WT901_SAVE_RESET = 0x0001, // 恢复出厂值
} WT901_SaveRegTypeDef;

/**
 * @brief CALSW 寄存器可选的写入值
 */
typedef enum
{
    WT901_CALSW_NORMAL = 0x0000, // 正常工作模式
    WT901_CALSW_ACCEL_CALIB = 0x0101, // 自动加速度计校准
    WT901_CALSW_ANGLE_CALIB = 0x0008, // 设置角度参考
} WT901_CalswRegTypeDef;

/**
 * @brief KEY 寄存器可选的写入值
 */
typedef enum
{
    WT901_KEY_UNLOCK = 0xB588,
} WT901_KeyRegTypeDef;

/**
 * @brief BAUD 寄存器可选的写入值
 */
typedef enum
{
    WT901_BAUD_4800 = 0x0001,
    WT901_BAUD_9600 = 0x0002,
    WT901_BAUD_19200 = 0x0003,
    WT901_BAUD_38400 = 0x0004,
    WT901_BAUD_57600 = 0x0005,
    WT901_BAUD_115200 = 0x0006,
    WT901_BAUD_230400 = 0x0007,
} WT901_BAUDTypeDef;

/**
 * @brief READ 寄存器可选的写入值
 */
typedef enum
{
    WT901_READ_READ = 0x0000,
} WT901_ReadRegTypeDef;

/* <-------------------通信协议相关-------------------> */
/**
 * @brief WT901 通信协议中的数据类型对应字节
 */
typedef enum
{
    WT901_DATA_TIME = 0x50, // 时间
    WT901_DATA_ACCEL = 0x51, // 加速度
    WT901_DATA_GYRO = 0x52, // 角速度
    WT901_DATA_ANGLE = 0x54, // 角度
    WT901_DATA_READ = 0x5F, // 读取
} WT901_DataTypeDef;

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
struct WT901_Gyro
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
    int16_t roll; // 横滚轴
    int16_t pitch; // 俯仰轴
    int16_t yaw; // 航向轴
};

/**
 * @brief 环形缓冲区
 */
typedef struct
{
    volatile uint8_t cirbuf[WT901_CIR_SIZE]; // 环形接收缓冲区
    volatile uint16_t head; // 头指针
    volatile uint16_t tail; // 尾指针
} WT901_CircularBuffer;

// 外部变量声明
extern volatile uint8_t g_wt901_buf[WT901_BUF_SIZE]; // DMA 接收缓冲区
extern volatile WT901_CircularBuffer g_wt901_cirbuf; //环形缓冲区
extern const uint8_t WT901_HEADER[];
extern volatile uint32_t g_wt901_overflow_count; //丢字节计数

/* <---------------------函数相关---------------------> */
/**
 * @brief 开启对 WT901 的数据接收
 *
 * @retval HAL_StatusTypeDef 开启结果
 */
__STATIC_INLINE HAL_StatusTypeDef WT901_StartReceive(void)
{
    return HAL_UARTEx_ReceiveToIdle_DMA(&WT901_UART, (uint8_t*)g_wt901_buf, WT901_BUF_SIZE);
}

/**
 * @brief 重启 WT901
 * 
 * @return HAL_StatusTypeDef 传输状态
 */
HAL_StatusTypeDef WT901_Restart(void);

/**
 * @brief 恢复 WT901 出厂设置
 * 
 * @return HAL_StatusTypeDef 传输状态
 */
HAL_StatusTypeDef WT901_Reset(void);

/**
 * @brief 修改 WT901 波特率
 * 
 * @param Baud 目标波特率
 *          @arg WT901_BAUD_4800
 *          @arg WT901_BAUD_9600
 *          @arg WT901_BAUD_19200
 *          @arg WT901_BAUD_38400
 *          @arg WT901_BAUD_57600
 *          @arg WT901_BAUD_115200
 *          @arg WT901_BAUD_230400
 * @return HAL_StatusTypeDef 传输状态
 */
HAL_StatusTypeDef WT901_Baud_Modify(WT901_BAUDTypeDef Baud);

/**
 * @brief 读一次 WT901 数据
 * 
 * @note 仅在 WT901 设置为单次传输模式时使用
 * @return HAL_StatusTypeDef 传输状态
 */
HAL_StatusTypeDef WT901_Read(void);

/**
 * @brief 初始化 WT901，校准加速度传感器并设置角度参考
 * 
 * @return HAL_StatusTypeDef 传输状态
 */
HAL_StatusTypeDef WT901_Init(void);

/**
 * @brief buf缓冲区下个位置
 * 
 * @param index 当前索引
 * @retval uint16_t 下个索引
 */
uint16_t WT901_BufNext(uint16_t index);

/**
 * @brief 读取一个字节
 * 
 * @param data 读取到的字节
 * @retval bool 是否成功读取
 */
bool WT901_CirRead(uint8_t* data);

/**
 * @brief 写入一个字节
 * 
 * @param data 要写入的字节
 * @retval bool 写入是否成功
 */
bool WT901_CirWrite(uint8_t data);

#ifdef __cplusplus
}
#endif
