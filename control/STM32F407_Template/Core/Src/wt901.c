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
struct WT901_Palst g_wt901_palst = { 0 }; // 角速度
struct WT901_Angle g_wt901_angle = { 0 }; // 角度

int16_t g_wt901_temprature = 0; // 温度
int16_t g_wt901_version = 0; // 版本号

/* <----------------指令常量----------------> */
const uint8_t WT901_CMD_UNLOCK[5] = { WT901_HEADER_1, WT901_HEADER_2, WT901_REG_KEY, 0x88, 0xB5 }; // 解锁
const uint8_t WT901_CMD_SAVE[5] = { WT901_HEADER_1, WT901_HEADER_2, WT901_REG_SAVE, 0x00, 0x00 }; // 保存
const uint8_t WT901_CMD_READ_ACCEL[5] = { WT901_HEADER_1, WT901_HEADER_2, WT901_REG_READ, WT901_REG_ACC_X, 0x00 }; // 读取加速度
const uint8_t WT901_CMD_READ_PALST[5] = { WT901_HEADER_1, WT901_HEADER_2, WT901_REG_READ, WT901_REG_GX, 0x00 }; // 读取角速度
const uint8_t WT901_CMD_READ_ANGLE[5] = { WT901_HEADER_1, WT901_HEADER_2, WT901_REG_READ, WT901_REG_ANG_R, 0x00 }; // 读取角度

/* <-----------------缓冲区-----------------> */
uint8_t g_wt901_buf[WT901_BUF_SIZE] = { 0 }; // 串口接收缓冲区

/* <------------------函数------------------> */
/**
 * @brief 计算校验和
 * 
 * @param Data 传入的数组
 * @param Length 计算长度
 * @retval uint8_t 校验和
 * @note 内部没有越界校验，使用时注意不要越界！
 */
static uint8_t CheckSum(uint8_t* Data, uint32_t Length)
{
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
        if (__HAL_DMA_GET_FLAG(&WT901_DMA, WT901_DMA_HT_FLAG)) // 半传输中断
        {
            __HAL_DMA_CLEAR_FLAG(&WT901_DMA, WT901_DMA_HT_FLAG);
        }
        else // 传输完成中断
        {
        }
    }
    else
    {
    }
}
