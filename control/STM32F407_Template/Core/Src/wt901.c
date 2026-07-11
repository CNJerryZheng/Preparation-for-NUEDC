/**
 * @file        wt901.c
 * @author      Misybon
                JerryZheng
 * @brief       WT901通信驱动
 * @warning     注意数组越界风险
 * @date        2026-07-11
*/

#include "wt901.h"

/* <----------------数据变量----------------> */
struct WT901_Accel g_wt901_accel = { 0 }; // 加速度
struct WT901_Gyro g_wt901_gyro = { 0 }; // 角速度
struct WT901_Angle g_wt901_angle = { 0 }; // 角度

int16_t g_wt901_temperature = 0; // 温度
int16_t g_wt901_version = 0; // 版本号

volatile uint8_t g_wt901_buf[WT901_BUF_SIZE] = { 0 }; // DMA 接收缓冲区

/* <-----------------缓冲区-----------------> */

volatile WT901_CircularBuffer g_wt901_cirbuf = { { 0 }, 0, 0 }; // 初始化环形缓冲区
static volatile uint8_t s_wt901_framebuf[WT901_FRAME_SIZE];
static volatile uint16_t s_frame_pos = 0;
static uint16_t s_wt901_dma_last_pos = 0;
volatile uint32_t g_wt901_lose_count = 0, g_wt901_count = 0; //丢包计数和总包数

/* <------------------函数------------------> */
/**
 * @brief 计算校验和
 * 
 * @param Data 传入的数组
 * @param Length 包长度
 * @retval bool 数据包是否正确
 * @warning 内部没有越界校验，使用时注意不要越界！
 */
static bool CheckSum(const volatile uint8_t* Data, uint32_t Length)
{
    if (Data == nullptr) // 指针判空
    {
        return false;
    }
    if (Data[0] != WT901_FRAME_HEADER) //包头不对
    {
        return false;
    }

    uint32_t sum = 0; // 计算校验和
    for (uint32_t index = 0; index < Length - 1; index++)
    {
        sum += Data[index];
    }
    return (uint8_t)sum == Data[Length - 1];
}

/**
 * @brief 缓冲区下个位置
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
 * @brief 数据包重设位置，传完一个设置到下个包头开始
 */
static void WT901_Reset_Frame_Pos(void)
{
    uint16_t next_pos = 1;
    while (next_pos < WT901_FRAME_SIZE)
    {
        if (s_wt901_framebuf[next_pos] == WT901_FRAME_HEADER)
        {
            break;
        }
        next_pos = -~next_pos;
    }

    for (uint16_t index = 0; index < WT901_FRAME_SIZE - next_pos; index = -~index)
    {
        s_wt901_framebuf[index] = s_wt901_framebuf[next_pos + index];
    }
    s_frame_pos = WT901_FRAME_SIZE - next_pos;
    return;
}

/**
 * @brief 环形缓冲区写入数据包
 * 
 * @retval bool 数据包是否写入成功
 */
static bool WT901_CirWrite_Frame(void)
{
    uint16_t write_pos;

    if (!CheckSum(s_wt901_framebuf, WT901_FRAME_SIZE))
    {
        WT901_Reset_Frame_Pos();
        return false;
    }

    if (WT901_CirNext(g_wt901_cirbuf.tail) == g_wt901_cirbuf.head)
    {
        s_frame_pos = 0;
        return false;
    }

    s_wt901_framebuf[WT901_FRAME_SIZE - 1] = WT901_FRAME_TAILER;
    write_pos = g_wt901_cirbuf.tail;
    for (uint16_t index = 0; index < WT901_FRAME_SIZE; index = -~index)
    {
        g_wt901_cirbuf.cirbuf[write_pos] = s_wt901_framebuf[index];
        write_pos = WT901_CirNext(write_pos);
    }
    g_wt901_cirbuf.tail = write_pos;
    s_frame_pos = 0;
    return true;
}

void WT901_CirWrite_Data(uint8_t data)
{
    s_wt901_framebuf[s_frame_pos] = data;
    s_frame_pos = -~s_frame_pos;
    if (s_frame_pos == WT901_FRAME_SIZE)
    {
        if (WT901_CirWrite_Frame())
        {
            g_wt901_count = -~g_wt901_count;
        }
        else
        {
            g_wt901_lose_count = -~g_wt901_lose_count;
            g_wt901_count = -~g_wt901_count;
        }
    }
    return;
}

bool WT901_CirRead(uint8_t* Data, uint32_t Length)
{
    if ((Data == nullptr) || (Length != WT901_FRAME_SIZE))
    {
        return false;
    }
    if (g_wt901_cirbuf.head == g_wt901_cirbuf.tail)
    {
        return false;
    }
    if (g_wt901_cirbuf.cirbuf[g_wt901_cirbuf.head] != WT901_FRAME_HEADER || g_wt901_cirbuf.cirbuf[(g_wt901_cirbuf.head + Length - 1) % WT901_CIR_SIZE] != WT901_FRAME_TAILER)
    {
        return false;
    }
    uint16_t read_pos = g_wt901_cirbuf.head;
    for (int index = 0; index < Length; index = -~index)
    {
        Data[index] = g_wt901_cirbuf.cirbuf[read_pos];
        read_pos = WT901_CirNext(read_pos);
    }
    g_wt901_cirbuf.head = read_pos;
    return true;
}

/**
 * @brief 中断回调处理dma数据
 */
static void WT901_ProcessDmaRx(uint16_t size)
{
    uint16_t write_pos = (size >= WT901_BUF_SIZE) ? 0U : size;

    while (s_wt901_dma_last_pos != write_pos)
    {
        WT901_CirWrite_Data(g_wt901_buf[s_wt901_dma_last_pos]);
        s_wt901_dma_last_pos = WT901_BufNext(s_wt901_dma_last_pos);
    }
}

/**
 * @brief HAL callback for USART Receive-to-IDLE DMA events.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == &WT901_UART)
    {
        WT901_ProcessDmaRx(Size);
    }
}

/**
 * @brief WT901 写寄存器
 * 
 * @param Reg 目标寄存器
 * @param Value 寄存器值
 * @retval HAL_StatusTypeDef 传输状态
 */
static HAL_StatusTypeDef WT901_WriteReg(WT901_RegTypeDef Reg, int16_t Value)
{
    // 进行拼帧
    uint8_t frame[5];
    frame[0] = WT901_HEADER_1;
    frame[1] = WT901_HEADER_2;
    frame[2] = (uint8_t)Reg;
    frame[3] = (uint8_t)(Value & 0xFF);
    frame[4] = (uint8_t)((Value >> 8) & 0xFF);

    // 传输
    return HAL_UART_Transmit(&WT901_UART, frame, sizeof(frame), 100);
}

HAL_StatusTypeDef WT901_Restart(void)
{
    HAL_StatusTypeDef status;

    // 解锁
    status = WT901_WriteReg(WT901_REG_KEY, (int16_t)WT901_KEY_UNLOCK);
    if (status != HAL_OK)
    {
        return status;
    }

    // 重启
    return WT901_WriteReg(WT901_REG_SAVE, WT901_SAVE_RESTART);
}

HAL_StatusTypeDef WT901_Reset(void)
{
    HAL_StatusTypeDef status;

    // 解锁
    status = WT901_WriteReg(WT901_REG_KEY, (int16_t)WT901_KEY_UNLOCK);
    if (status != HAL_OK)
    {
        return status;
    }

    // 重置
    return WT901_WriteReg(WT901_REG_SAVE, WT901_SAVE_RESET);
}

static HAL_StatusTypeDef WT901_Accel_Calibrate(void)
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
    status = WT901_WriteReg(WT901_REG_CALSW, (int16_t)WT901_CALSW_ACCEL_CALIB);
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

static HAL_StatusTypeDef WT901_Angle_Calibrate(void)
{
    // 解锁
    HAL_StatusTypeDef status = WT901_WriteReg(WT901_REG_KEY, (int16_t)WT901_KEY_UNLOCK);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    // 设置角度参考
    status = WT901_WriteReg(WT901_REG_CALSW, (int16_t)WT901_CALSW_ANGLE_CALIB);
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

    // 拼接写入值
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
    status = WT901_WriteReg(WT901_REG_RSW, (int16_t)(val & 0x7FF));
    if (status != HAL_OK)
    {
        return status;
    }

    // 保存
    return WT901_WriteReg(WT901_REG_SAVE, WT901_SAVE_SAVE);
}

HAL_StatusTypeDef WT901_Baud_Modify(WT901_BAUDTypeDef Baud)
{
    // 解锁
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

/**
 * @brief 设置输出速率
 * 
 * @return HAL_StatusTypeDef 传输状态
 */
static HAL_StatusTypeDef WT901_OutputRate_Modify(void)
{
    HAL_StatusTypeDef status;

    // 解锁
    status = WT901_WriteReg(WT901_REG_KEY, (int16_t)(WT901_KEY_UNLOCK));
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    // 设置写入值
    uint32_t val = 0;
#ifdef WT901_RRATE_0_2HZ
    val = 0x01;
#elifdef WT901_RRATE_0_5HZ
    val = 0x02;
#elifdef WT901_RRATE_1HZ
    val = 0x03;
#elifdef WT901_RRATE_2HZ
    val = 0x04;
#elifdef WT901_RRATE_5HZ
    val = 0x05;
#elifdef WT901_RRATE_10HZ
    val = 0x06;
#elifdef WT901_RRATE_20HZ
    val = 0x07;
#elifdef WT901_RRATE_50HZ
    val = 0x08;
#elifdef WT901_RRATE_100HZ
    val = 0x09;
#elifdef WT901_RRATE_200HZ
    val = 0x0B;
#elifdef WT901_RRATE_SINGLE
    val = 0x0C;
#elifdef WT901_RRATE_NO
    val = 0x0D;
#else
#error WT901 RRATE Error!!!
#endif

    // 写入寄存器
    status = WT901_WriteReg(WT901_REG_RRATE, (int16_t)val);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(200);

    // 保存
    return WT901_WriteReg(WT901_REG_SAVE, WT901_SAVE_SAVE);
}

static HAL_StatusTypeDef WT901_GyroRange_Modify(void)
{
}

HAL_StatusTypeDef WT901_Read(void)
{
    return WT901_WriteReg(WT901_REG_READ, WT901_READ_READ);
}

HAL_StatusTypeDef WT901_Init(void)
{
    // 校准加速度
    HAL_StatusTypeDef status = WT901_Accel_Calibrate();
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

    // 修改输出速率
    status = WT901_OutputRate_Modify();
    if (status != HAL_OK)
    {
        return status;
    }

    // 设置角度参考
    return WT901_Angle_Calibrate();
}
