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
 * @brief 
 * 
 * @param Reg 
 * @param Value 
 */
static void WT901_WriteReg(WT901_RegTypeDef Reg, int16_t Value)
{
    // 进行拼帧
    static uint8_t frame[5];
    frame[0] = WT901_HEADER_1;
    frame[1] = WT901_HEADER_2;
    frame[2] = (uint8_t)Reg;
    frame[3] = (uint8_t)(Value & 0xFF);
    frame[4] = (uint8_t)((Value >> 8) & 0xFF);

    // 传输
    HAL_UART_Transmit_DMA(&WT901_UART, frame, sizeof(frame));
}

/**
 * @brief 进行 WT901 的加速度校准
 */
void WT901_Accel_Callibrate(void)
{
    HAL_Delay(200);
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

/**
 * @brief 重写串口接收中断回调函数
 * 
 * @param huart 串口句柄
 * @param Size 接收的数据长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == &WT901_UART)
    {
    }
    else
    {
    }
}
