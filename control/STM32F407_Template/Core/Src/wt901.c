/**
 * @file        wt901.c
 * @brief       WT901通信驱动
 * @warning     注意数组越界风险
 * @author      Misybon
 * @date        2026-07-07
*/

#include "wt901.h"

/* <----------------数据变量----------------> */
struct WT901_Accel g_wt901_accel = { 0 }; // 加速度
struct WT901_Gyro g_wt901_gyro = { 0 }; // 角速度
struct WT901_Angle g_wt901_angle = { 0 }; // 角度

int16_t g_wt901_temperature = 0; // 温度
int16_t g_wt901_version = 0; // 版本号

static uint8_t s_wt901_frame[5] = { 0 };

/* <----------------指令常量----------------> */
const uint8_t WT901_HEADER[] = { WT901_HEADER_1, WT901_HEADER_2 };

/* <-----------------缓冲区-----------------> */

volatile WT901_CircularBuffer g_wt901_cirbuf = { { 0 }, 0, 0 }; // 初始化环形缓冲区
volatile uint32_t g_wt901_overflow_count = 0; //丢字节计数

/* <------------------函数------------------> */
/**
 * @brief buf缓冲区下个位置
 * 
 * @param index 当前索引
 * @retval uint16_t 下个索引
 */
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

/**
 * @brief 写入一个字节
 * 
 * @param data 要写入的字节
 * @retval bool 写入是否成功
 */
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

/**
 * @brief 读取一个字节
 * 
 * @param data 读取到的字节
 * @retval bool 是否成功读取
 */
bool WT901_CirRead(uint8_t* data)
{
    if (data == NULL)
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

/**
 * @brief 校准 WT901 加速度传感器
 * 
 * @return HAL_StatusTypeDef 传输状态
 */
static HAL_StatusTypeDef WT901_Accel_Callibrate(void)
{
    HAL_StatusTypeDef status;

    status = WT901_WriteReg(WT901_REG_KEY, (int16_t)WT901_KEY_UNLOCK);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    status = WT901_WriteReg(WT901_REG_CALSW, (int16_t)WT901_CALSW_ACCEL_CALLIB);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(4000);

    status = WT901_WriteReg(WT901_REG_CALSW, (int16_t)WT901_CALSW_NORMAL);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(100);

    return WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_SAVE);
}

/**
 * @brief 设置 WT901 角度参考
 * 
 * @return HAL_StatusTypeDef 传输状态
 */
static HAL_StatusTypeDef WT901_Angle_Callibrate(void)
{
    HAL_StatusTypeDef status;

    status = WT901_WriteReg(WT901_REG_KEY, (int16_t)WT901_KEY_UNLOCK);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    status = WT901_WriteReg(WT901_REG_CALSW, (int16_t)WT901_CALSW_ANGLE_CALLIB);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(3000);

    return WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_SAVE);
}

HAL_StatusTypeDef WT901_Baud_Modify(WT901_BAUDTypeDef Baud)
{
    HAL_StatusTypeDef status;

    status = WT901_WriteReg(WT901_REG_KEY, (int16_t)WT901_KEY_UNLOCK);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    status = WT901_WriteReg(WT901_REG_BAUD, (int16_t)Baud);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    status = WT901_WriteReg(WT901_REG_KEY, (int16_t)WT901_KEY_UNLOCK);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    return status = WT901_WriteReg(WT901_REG_SAVE, WT901_SAVE_SAVE);
}

HAL_StatusTypeDef WT901_Init(void)
{
    HAL_StatusTypeDef status;

    status = WT901_Accel_Callibrate();
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
