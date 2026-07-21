/**
 * @file        wt901.h
 * @author      Misybon
                JerryZheng
 * @brief       WT901通信驱动
 * @warning     注意数组越界风险
 * @date        2026-07-11
*/

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "wt901_config.h"

typedef enum { WT901_OK = 0, WT901_ERROR = 1 } WT901_StatusTypeDef;

#ifdef __cplusplus
extern "C" {
#endif

/* <--------------------寄存器相关--------------------> */
/**
 * @brief WT901 寄存器地址
 */
typedef enum
{
    WT901_REG_SAVE = 0x00, // 保存/重启/恢复出厂
    WT901_REG_CALSW = 0x01, // 校准模式
    WT901_REG_RSW = 0x02, // 输出内容
    WT901_REG_RRATE = 0x03, // 输出速率
    WT901_REG_BAUD = 0x04, // 串口波特率
    WT901_REG_GYRORANGE = 0x20, // 陀螺仪量程
    WT901_REG_ACCRANGE = 0x23, // 加速度计量程
    WT901_REG_SLEEP = 0x22, // 休眠
    WT901_REG_AXIS6 = 0x24, // 算法
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
    WT901_CALSW_ACCEL_CALIB = 0x0001, // 自动加速度计校准
    WT901_CALSW_YAW_CALIB = 0x0004, // 航向轴校准
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

/**
 * @brief AXIS6 寄存器可选的写入值
 */
typedef enum
{
    WT901_AXIS6_9 = 0x0000, // 九轴算法
    WT901_AXIS6_6 = 0x0001, // 六轴算法
} WT901_Axis6RegTypeDef;

/* <-------------------通信协议相关-------------------> */
/**
 * @brief WT901 通信协议中的数据类型对应字节
 */
typedef enum
{
    WT901_DATA_TIME = 0x50, // 时间
    WT901_DATA_ACCEL = 0x51, // 加速度
    WT901_DATA_GYRO = 0x52, // 角速度
    WT901_DATA_ANGLE = 0x53, // 角度
    WT901_DATA_MAG = 0x54, // 磁场
    WT901_DATA_PORT = 0x55, // 端口状态
    WT901_DATA_PRESS = 0x56, // 气压高度
    WT901_DATA_GPS = 0x57, // 经纬度
    WT901_DATA_VELOCITY = 0x58, // 地速
    WT901_DATA_QUATER = 0x59, // 四元数
    WT901_DATA_GSA = 0x5A, // 定位精度
    WT901_DATA_READ = 0x5F, // 读取
} WT901_DataTypeDef;

/* <---------------------变量相关---------------------> */
// 变量结构体
/**
 * @brief 加速度，单位m/s²
 */
struct WT901_Accel
{
    float x;
    float y;
    float z;
};

/**
 * @brief 角速度，单位°/s
 */
struct WT901_Gyro
{
    float x;
    float y;
    float z;
};

/**
 * @brief 角度，单位°
 */
struct WT901_Angle
{
    float roll; // 横滚轴
    float pitch; // 俯仰轴
    float yaw; // 航向轴
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
extern struct WT901_Accel g_wt901_accel;
extern struct WT901_Gyro g_wt901_gyro;
extern struct WT901_Angle g_wt901_angle;
extern int16_t g_wt901_temperature;
extern int16_t g_wt901_version;
extern volatile uint8_t g_wt901_buf[WT901_BUF_SIZE]; // DMA 接收缓冲区
extern volatile WT901_CircularBuffer g_wt901_cirbuf; //环形缓冲区
extern volatile uint32_t g_wt901_lose_count, g_wt901_count; //丢包计数和总包数

/* <---------------------函数相关---------------------> */
/**
 * @brief 开启对 WT901 的数据接收
 *
 * @retval WT901_StatusTypeDef 开启结果
 */
WT901_StatusTypeDef WT901_StartReceive(void);
WT901_StatusTypeDef WT901_StopReceive(void);

/**
 * @brief 重启 WT901
 * 
 * @return WT901_StatusTypeDef 传输状态
 */
WT901_StatusTypeDef WT901_Restart(void);

/**
 * @brief 恢复 WT901 出厂设置
 * 
 * @return WT901_StatusTypeDef 传输状态
 */
WT901_StatusTypeDef WT901_Reset(void);

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
 * @return WT901_StatusTypeDef 传输状态
 */
WT901_StatusTypeDef WT901_Baud_Modify(WT901_BAUDTypeDef Baud);

/**
 * @brief 读一次 WT901 数据
 * 
 * @note 仅在 WT901 设置为单次传输模式时使用
 * @return WT901_StatusTypeDef 传输状态
 */
WT901_StatusTypeDef WT901_Read(void);

/**
 * @brief 初始化 WT901
 * 
 * @return WT901_StatusTypeDef 传输状态
 */
WT901_StatusTypeDef WT901_Init(void);

/**
 * @brief buf缓冲区下个位置
 * 
 * @param index 当前索引
 * @retval uint16_t 下个索引
 */
uint16_t WT901_BufNext(uint16_t index);

/**
 * @brief 读取环形缓冲区数据包
 * 
 * @param Data 数据存放位置
 * @param Length 读取长度
 * @retval bool 是否成功读取
 */
bool WT901_CirRead(uint8_t* Data, uint32_t Length);

/**
 * @brief 写入一个字节
 * 
 * @param data 要写入的字节
 * @retval bool 写入是否成功
 */
void WT901_CirWrite_Data(uint8_t data);

/**
 * @brief 分析 WT901 数据帧
 * 
 * @retval 分析是否成功
 */
bool WT901_AnalyzeData(void);

#ifdef __cplusplus
}
#endif

