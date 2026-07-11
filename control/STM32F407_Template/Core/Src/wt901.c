/**
 * @file        wt901.c
 * @brief       WT901通信驱动
 * @warning     注意数组越界风险
 * @author      Misybon
                JerryChen
 * @date        2026-07-07
*/

#include "wt901.h"

/* <----------------数据变量----------------> */
struct WT901_Accel g_wt901_accel = { 0 }; // 加速度
struct WT901_Gyro g_wt901_gyro = { 0 }; // 角速度
struct WT901_Angle g_wt901_angle = { 0 }; // 角度

int16_t g_wt901_temperature = 0; // 温度
int16_t g_wt901_version = 0; // 版本号

static uint8_t s_wt901_frame[5] = { 0 }; // WT901 数据帧

/* <----------------指令常量----------------> */
const uint8_t WT901_HEADER[] = { WT901_HEADER_1, WT901_HEADER_2 };

/* <-----------------缓冲区-----------------> */

volatile WT901_CircularBuffer g_wt901_cirbuf = { { 0 }, 0, 0 }; // 初始化环形缓冲区
volatile uint32_t g_wt901_overflow_count = 0; //丢字节计数

/* <------------------函数------------------> */
uint16_t WT901_BufNext(uint16_t index)
{
    return (index + 1) % WT901_BUF_SIZE;
}

/**
 * @brief 环形缓冲区下个位置
 * 
 * @param index 当前索引
 * @retval uint16_t 下个索引
 */
static uint16_t WT901_CirNext(uint16_t index)
{
    return (index + 1) % WT901_CIR_SIZE;
}

bool WT901_CirWrite(uint8_t data)
{
    uint16_t next = WT901_CirNext(g_wt901_cirbuf.tail);
    if (next == g_wt901_cirbuf.head) // 缓冲区满
    {
        g_wt901_overflow_count = -~g_wt901_overflow_count;
        return false;
    }
    g_wt901_cirbuf.cirbuf[g_wt901_cirbuf.tail] = data;
    g_wt901_cirbuf.tail = next;
    return true;
}

bool WT901_CirRead(uint8_t* data)
{
    if (data == nullptr)
    {
        return false;
    }

    if (g_wt901_cirbuf.head == g_wt901_cirbuf.tail)
    {
        return false;
    }

    *data = g_wt901_cirbuf.cirbuf[g_wt901_cirbuf.head];

    g_wt901_cirbuf.head = WT901_CirNext(g_wt901_cirbuf.head);

    return true;
}

/**
 * @brief WT901 写寄存器
 * 
 * @param Reg 目标寄存器
 * @param Value 寄存器值
 * @retval HAL_StatusTypeDef 状态
 */
static HAL_StatusTypeDef WT901_WriteReg(WT901_RegTypeDef Reg, int16_t Value)
{
    // 进行拼帧
    s_wt901_frame[0] = WT901_HEADER_1;
    s_wt901_frame[1] = WT901_HEADER_2;
    s_wt901_frame[2] = (uint8_t)Reg;
    s_wt901_frame[3] = (uint8_t)(Value & 0xFF);
    s_wt901_frame[4] = (uint8_t)((Value >> 8) & 0xFF);

    // 传输
    return HAL_UART_Transmit_DMA(&WT901_UART, s_wt901_frame, sizeof(s_wt901_frame));
}

HAL_StatusTypeDef WT901_Restart(void)
{
    return WT901_WriteReg(WT901_REG_SAVE, WT901_SAVE_RESTART);
}

HAL_StatusTypeDef WT901_Reset(void)
{
    return WT901_WriteReg(WT901_REG_SAVE, WT901_SAVE_RESET);
}

static HAL_StatusTypeDef WT901_Accel_Callibrate(void)
{
    HAL_StatusTypeDef status;

    // 解锁
    status = WT901_WriteReg(WT901_REG_KEY, (int16_t)WT901_KEY_UNLOCK);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    // 校准加速度
    status = WT901_WriteReg(WT901_REG_CALSW, (int16_t)WT901_CALSW_ACCEL_CALLIB);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(4000);

    // 退出校准模式
    status = WT901_WriteReg(WT901_REG_CALSW, (int16_t)WT901_CALSW_NORMAL);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(100);

    // 保存
    return WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_SAVE);
}

static HAL_StatusTypeDef WT901_Angle_Callibrate(void)
{
    // 解锁
    HAL_StatusTypeDef status = WT901_WriteReg(WT901_REG_KEY, (int16_t)WT901_KEY_UNLOCK);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    // 设置角度参考
    status = WT901_WriteReg(WT901_REG_CALSW, (int16_t)WT901_CALSW_ANGLE_CALLIB);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(3000);

    // 保存
    return WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_SAVE);
}

static HAL_StatusTypeDef WT901_Output_Modify(void)
{
    // 解锁
    HAL_StatusTypeDef status = WT901_WriteReg(WT901_REG_KEY, (int16_t)(WT901_KEY_UNLOCK));
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    uint32_t val = 0;

#if defined(WT901_TIME_OUT)
    val |= 0x01 << 0;
#endif

#if defined(WT901_ACC_OUT)
    val |= 0x01 << 1;
#endif

#if defined(WT901_GYRO_OUT)
    val |= 0x01 << 2;
#endif

#if defined(WT901_ANGLE_OUT)
    val |= 0x01 << 3;
#endif

#if defined(WT901_MAG_OUT)
    val |= 0x01 << 4;
#endif

#if defined(WT901_PORT_OUT)
    val |= 0x01 << 5;
#endif

#if defined(WT901_PRESS_OUT)
    val |= 0x01 << 6;
#endif

#if defined(WT901_GPS_OUT)
    val |= 0x01 << 7;
#endif

#if defined(WT901_VELOCITY_OUT)
    val |= 0x01 << 8;
#endif

#if defined(WT901_QUATER_OUT)
    val |= 0x01 << 9;
#endif

#if defined(WT901_GSA_OUT)
    val |= 0x01 << 10;
#endif

    // 设置输出内容
    status = WT901_WriteReg(WT901_REG_RSW, (int16_t)(val & 0xFF));
    if (status != HAL_OK)
    {
        return status;
    }

    // 保存
    return WT901_WriteReg(WT901_REG_SAVE, WT901_SAVE_SAVE);
}

HAL_StatusTypeDef WT901_Baud_Modify(WT901_BAUDTypeDef Baud)
{
    /// 解锁
    HAL_StatusTypeDef status = WT901_WriteReg(WT901_REG_KEY, (int16_t)WT901_KEY_UNLOCK);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    // 设置波特率
    status = WT901_WriteReg(WT901_REG_BAUD, (int16_t)Baud);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    // 解锁
    status = WT901_WriteReg(WT901_REG_KEY, (int16_t)WT901_KEY_UNLOCK);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    // 保存
    return WT901_WriteReg(WT901_REG_SAVE, WT901_SAVE_SAVE);
}

HAL_StatusTypeDef WT901_Init(void)
{
    // 校准加速度
    HAL_StatusTypeDef status = WT901_Accel_Callibrate();
    if (status != HAL_OK)
    {
        return status;
    }

    // 修改输出内容
    status = WT901_Output_Modify();
    if (status != HAL_OK)
    {
        return status;
    }

    return WT901_Angle_Callibrate();
}

/**
 * @brief 计算校验和
 * 
 * @param Data 传入的数组
 * @param Length 计算长度
 * @retval uint8_t 校验和
 * @warning 内部没有越界校验，使用时注意不要越界！
 */
static uint8_t CheckSum(const uint8_t* Data, uint32_t Length)
{
    if (Data == nullptr) // 指针判空
    {
        return 0;
    }

    // 计算校验和
    uint32_t sum = 0;
    for (uint32_t index = 0; index < Length; index++)
    {
        sum += Data[index];
    }
    return sum;
}
