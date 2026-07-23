/**
 * @file        wt901.c
 * @author      Misybon
 *              JerryZheng
 * @brief       WT901 姿态传感器设备驱动
 * @note        保留原 F407 的帧缓存及姿态换算逻辑，UART 接收改由 MSPM0 BSP 提供
 * @date        2026-07-21
 */

#include "wt901.h"
#include "board.h"
#include "bsp_uart.h"

/* <----------------数据变量----------------> */
struct WT901_Accel g_wt901_accel = { 0.0f, 0.0f, 0.0f };
struct WT901_Gyro g_wt901_gyro = { 0.0f, 0.0f, 0.0f };
struct WT901_Angle g_wt901_angle = { 0.0f, 0.0f, 0.0f };
int16_t g_wt901_temperature = 0;
int16_t g_wt901_version = 0;
volatile uint8_t g_wt901_buf[WT901_BUF_SIZE] = { 0U };
volatile WT901_CircularBuffer g_wt901_cirbuf = { { 0U }, 0U, 0U };
volatile uint32_t g_wt901_lose_count = 0U;
volatile uint32_t g_wt901_count = 0U;
volatile uint32_t g_wt901_rx_byte_count = 0U;
volatile uint32_t g_wt901_checksum_error_count = 0U;
volatile uint32_t g_wt901_parse_count = 0U;
/** @brief 已成功解析的角度帧数量。 */
volatile uint32_t g_wt901_angle_update_count = 0U;
volatile uint32_t g_wt901_unknown_frame_count = 0U;
volatile uint8_t g_wt901_last_rx_byte = 0U;
volatile uint8_t g_wt901_last_frame_type = 0U;
volatile uint32_t g_wt901_init_stage = 0U;
static uint8_t s_wt901_raw_data[WT901_FRAME_SIZE] = { 0U };
static uint8_t s_wt901_framebuf[WT901_FRAME_SIZE] = { 0U };
static uint16_t s_frame_pos = 0U;
static bool s_wt901_version_obtained = false;
static bool s_wt901_receiving = false;

static WT901_StatusTypeDef WT901_Output_Modify(void);
static WT901_StatusTypeDef WT901_OutputRate_Modify(void);
static WT901_StatusTypeDef WT901_Axis6_Modify(void);

/**
 * @brief WT901 配置帧之间的短暂等待
 * @note 不依赖 SysTick，避免在系统节拍尚未稳定时阻塞初始化。
 */
static void WT901_Delay(uint32_t ms)
{
    delay_ms(ms);
}

/**
 * @brief 校验 WT901 原始帧校验和
 * @param data 十一字节原始数据帧
 * @retval bool 数据帧是否正确
 */
static bool WT901_CheckSum(const uint8_t* data)
{
    uint8_t sum = 0U;
    if ((data == 0) || (data[0] != WT901_FRAME_HEADER))
    {
        return false;
    }
    for (uint32_t index = 0U; index < (WT901_FRAME_SIZE - 1U); index++)
    {
        sum = (uint8_t)(sum + data[index]);
    }
    return sum == data[WT901_FRAME_SIZE - 1U];
}

/**
 * @brief 获取环形帧缓冲区的下一个索引
 * @param index 当前索引
 * @retval uint16_t 下一个索引
 */
static uint16_t WT901_CirNext(uint16_t index)
{
    return (uint16_t)((index + 1U) % WT901_CIR_SIZE);
}

/**
 * @brief 在校验失败时寻找可能的下一帧帧头
 */
static void WT901_ResetFramePos(void)
{
    uint16_t next_pos = 1U;
    while ((next_pos < WT901_FRAME_SIZE) && (s_wt901_framebuf[next_pos] != WT901_FRAME_HEADER))
    {
        next_pos++;
    }
    if (next_pos == WT901_FRAME_SIZE)
    {
        s_frame_pos = 0U;
        return;
    }
    for (uint16_t index = 0U; (next_pos + index) < WT901_FRAME_SIZE; index++)
    {
        s_wt901_framebuf[index] = s_wt901_framebuf[next_pos + index];
    }
    s_frame_pos = (uint16_t)(WT901_FRAME_SIZE - next_pos);
}

/**
 * @brief 将一帧校验成功的数据写入环形缓冲区
 * @retval bool 是否写入成功
 */
static bool WT901_CirWriteFrame(void)
{
    uint16_t write_pos;
    if (!WT901_CheckSum(s_wt901_framebuf))
    {
        g_wt901_checksum_error_count++;
        WT901_ResetFramePos();
        return false;
    }
    write_pos = g_wt901_cirbuf.tail;
    for (uint16_t index = 0U; index < WT901_FRAME_SIZE; index++)
    {
        uint16_t next = WT901_CirNext(write_pos);
        if (next == g_wt901_cirbuf.head)
        {
            s_frame_pos = 0U;
            return false;
        }
        g_wt901_cirbuf.cirbuf[write_pos] = s_wt901_framebuf[index];
        write_pos = next;
    }
    g_wt901_cirbuf.tail = write_pos;
    s_frame_pos = 0U;
    return true;
}

/**
 * @brief 读取小端有符号十六位数值
 * @param data 指向低字节
 * @retval int16_t 转换后的数值
 */
static int16_t WT901_ReadS16(const uint8_t* data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

/**
 * @brief 向 WT901 写入一个寄存器
 * @param reg 目标寄存器
 * @param value 十六位寄存器值
 * @retval WT901_StatusTypeDef 发送状态
 */
static WT901_StatusTypeDef WT901_WriteReg(WT901_RegTypeDef reg, int16_t value)
{
    uint8_t frame[5];

    frame[0] = WT901_HEADER_1;
    frame[1] = WT901_HEADER_2;
    frame[2] = (uint8_t)reg;
    frame[3] = (uint8_t)((uint16_t)value & 0x00FFU);
    frame[4] = (uint8_t)(((uint16_t)value >> 8U) & 0x00FFU);
    return BSP_UART_WT901_Transmit(frame, sizeof(frame)) ? WT901_OK : WT901_ERROR;
}

/**
 * @brief 解锁 WT901 配置寄存器
 * @retval WT901_StatusTypeDef 发送状态
 */
static WT901_StatusTypeDef WT901_Unlock(void)
{
    return WT901_WriteReg(WT901_REG_KEY, (int16_t)WT901_KEY_UNLOCK);
}

/**
 * @brief WT901 加速度计校准
 * @note 完整保留 F407 校准流程；仅在设备静止且明确需要重新校准时调用。
 */
static WT901_StatusTypeDef WT901_Accel_Calibrate(void)
{
    g_wt901_init_stage = 10U;
    if (WT901_Unlock() != WT901_OK)
    {
        return WT901_ERROR;
    }
    g_wt901_init_stage = 11U;
    WT901_Delay(200U);
    g_wt901_init_stage = 12U;
    if (WT901_WriteReg(WT901_REG_CALSW, (int16_t)WT901_CALSW_ACCEL_CALIB) != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(4000U);
    if (WT901_WriteReg(WT901_REG_CALSW, (int16_t)WT901_CALSW_NORMAL) != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(100U);
    return WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_SAVE);
}

/**
 * @brief WT901 角度参考校准
 * @note 完整保留 F407 校准流程；调用前必须将模块固定在期望参考姿态。
 */
static WT901_StatusTypeDef WT901_Angle_Calibrate(void)
{
    if (WT901_Unlock() != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(200U);
    if (WT901_WriteReg(WT901_REG_CALSW, (int16_t)WT901_CALSW_ANGLE_CALIB) != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(3000U);
    return WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_SAVE);
}

/**
 * @brief 设置 WT901 航向轴参考
 * @note 完整保留 F407 校准流程；调用前必须将云台朝向期望零点。
 */
static WT901_StatusTypeDef WT901_YawAxis_Calibrate(void)
{
    if (WT901_Unlock() != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(200U);
    if (WT901_WriteReg(WT901_REG_CALSW, (int16_t)WT901_CALSW_YAW_CALIB) != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(3000U);
    return WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_SAVE);
}

/**
 * @brief 初始化 WT901 软件接收状态
 */
WT901_StatusTypeDef WT901_Init(void)
{
    g_wt901_init_stage = 1U;
    g_wt901_cirbuf.head = 0U;
    g_wt901_cirbuf.tail = 0U;
    s_frame_pos = 0U;
    s_wt901_version_obtained = false;
    g_wt901_count = 0U;
    g_wt901_lose_count = 0U;
    g_wt901_rx_byte_count = 0U;
    g_wt901_checksum_error_count = 0U;
    g_wt901_parse_count = 0U;
    g_wt901_angle_update_count = 0U;
    g_wt901_unknown_frame_count = 0U;
    g_wt901_last_rx_byte = 0U;
    g_wt901_last_frame_type = 0U;
    g_wt901_init_stage = 2U;

    if ((WT901_Accel_Calibrate() != WT901_OK) || (WT901_Output_Modify() != WT901_OK) || (WT901_OutputRate_Modify() != WT901_OK) || (WT901_Baud_Modify(WT901_UART_BAUD_REG) != WT901_OK) || (WT901_Axis6_Modify() != WT901_OK) || (WT901_YawAxis_Calibrate() != WT901_OK) || (WT901_Angle_Calibrate() != WT901_OK))
    {
        return WT901_ERROR;
    }
    g_wt901_init_stage = 3U;
    return WT901_OK;
}

WT901_StatusTypeDef WT901_StartReceive(void)
{
    s_wt901_receiving = true;
    return WT901_OK;
}

WT901_StatusTypeDef WT901_StopReceive(void)
{
    s_wt901_receiving = false;
    return WT901_OK;
}

WT901_StatusTypeDef WT901_Restart(void)
{
    if (WT901_Unlock() != WT901_OK)
    {
        return WT901_ERROR;
    }
    return WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_RESTART);
}

WT901_StatusTypeDef WT901_Reset(void)
{
    if (WT901_Unlock() != WT901_OK)
    {
        return WT901_ERROR;
    }
    return WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_RESET);
}

WT901_StatusTypeDef WT901_Baud_Modify(WT901_BAUDTypeDef baud)
{
    if (WT901_Unlock() != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(200U);
    if (WT901_WriteReg(WT901_REG_BAUD, (int16_t)baud) != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(200U);
    if (WT901_Unlock() != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(200U);
    return WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_SAVE);
}

WT901_StatusTypeDef WT901_Read(void)
{
    return WT901_WriteReg(WT901_REG_READ, (int16_t)WT901_READ_READ);
}

/**
 * @brief 修改 WT901 输出内容
 * @note 保留 F407 的所有输出位配置宏。
 */
static WT901_StatusTypeDef WT901_Output_Modify(void)
{
    uint16_t output = 0U;

#ifdef WT901_TIME_OUT
    output |= (1U << 0U);
#endif
#ifdef WT901_ACC_OUT
    output |= (1U << 1U);
#endif
#ifdef WT901_GYRO_OUT
    output |= (1U << 2U);
#endif
#ifdef WT901_ANGLE_OUT
    output |= (1U << 3U);
#endif
#ifdef WT901_MAG_OUT
    output |= (1U << 4U);
#endif
#ifdef WT901_PORT_OUT
    output |= (1U << 5U);
#endif
#ifdef WT901_PRESS_OUT
    output |= (1U << 6U);
#endif
#ifdef WT901_GPS_OUT
    output |= (1U << 7U);
#endif
#ifdef WT901_VELOCITY_OUT
    output |= (1U << 8U);
#endif
#ifdef WT901_QUATER_OUT
    output |= (1U << 9U);
#endif
#ifdef WT901_GSA_OUT
    output |= (1U << 10U);
#endif

    if (WT901_Unlock() != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(200U);
    if (WT901_WriteReg(WT901_REG_RSW, (int16_t)(output & 0x07FFU)) != WT901_OK)
    {
        return WT901_ERROR;
    }
    if (WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_SAVE) != WT901_OK)
    {
        return WT901_ERROR;
    }
    return WT901_OK;
}

/** @brief 修改 WT901 输出速率 */
static WT901_StatusTypeDef WT901_OutputRate_Modify(void)
{
    uint16_t rate = 0U;

#if defined(WT901_RRATE_0_2HZ)
    rate = 0x01U;
#elif defined(WT901_RRATE_0_5HZ)
    rate = 0x02U;
#elif defined(WT901_RRATE_1HZ)
    rate = 0x03U;
#elif defined(WT901_RRATE_2HZ)
    rate = 0x04U;
#elif defined(WT901_RRATE_5HZ)
    rate = 0x05U;
#elif defined(WT901_RRATE_10HZ)
    rate = 0x06U;
#elif defined(WT901_RRATE_20HZ)
    rate = 0x07U;
#elif defined(WT901_RRATE_50HZ)
    rate = 0x08U;
#elif defined(WT901_RRATE_100HZ)
    rate = 0x09U;
#elif defined(WT901_RRATE_200HZ)
    rate = 0x0BU;
#elif defined(WT901_RRATE_SINGLE)
    rate = 0x0CU;
#elif defined(WT901_RRATE_NO)
    rate = 0x0DU;
#else
#error "WT901 RRATE Error"
#endif

    if (WT901_Unlock() != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(200U);
    if (WT901_WriteReg(WT901_REG_RRATE, (int16_t)rate) != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(200U);
    if (WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_SAVE) != WT901_OK)
    {
        return WT901_ERROR;
    }
    return WT901_OK;
}

/** @brief 设置 WT901 六轴或九轴算法 */
static WT901_StatusTypeDef WT901_Axis6_Modify(void)
{
#ifdef WT901_ALG_6
    const WT901_Axis6RegTypeDef algorithm = WT901_AXIS6_6;
#elif defined(WT901_ALG_9)
    const WT901_Axis6RegTypeDef algorithm = WT901_AXIS6_9;
#else
#error "WT901 Axis Error"
#endif

    if (WT901_Unlock() != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(200U);
    if (WT901_WriteReg(WT901_REG_AXIS6, (int16_t)algorithm) != WT901_OK)
    {
        return WT901_ERROR;
    }
    WT901_Delay(200U);
    if (WT901_WriteReg(WT901_REG_SAVE, (int16_t)WT901_SAVE_SAVE) != WT901_OK)
    {
        return WT901_ERROR;
    }
    return WT901_OK;
}

WT901_StatusTypeDef WT901_ApplyOutputConfig(void)
{
    if ((WT901_Output_Modify() != WT901_OK) || (WT901_OutputRate_Modify() != WT901_OK) || (WT901_Axis6_Modify() != WT901_OK))
    {
        return WT901_ERROR;
    }
    return WT901_OK;
}

/**
 * @brief 获取 DMA 接收缓冲区的下一个索引
 * @param index 当前索引
 * @retval uint16_t 下一个索引
 */
uint16_t WT901_BufNext(uint16_t index)
{
    return (uint16_t)((index + 1U) % WT901_BUF_SIZE);
}

/**
 * @brief 写入一个 UART 接收字节
 * @param data 接收到的字节
 */
void WT901_CirWrite_Data(uint8_t data)
{
    g_wt901_rx_byte_count++;
    g_wt901_last_rx_byte = data;
    if (!s_wt901_receiving)
    {
        return;
    }
    if ((s_frame_pos == 0U) && (data != WT901_FRAME_HEADER))
    {
        return;
    }
    s_wt901_framebuf[s_frame_pos++] = data;
    if (s_frame_pos == WT901_FRAME_SIZE)
    {
        if (WT901_CirWriteFrame())
        {
            g_wt901_count++;
        }
        else
        {
            g_wt901_lose_count++;
        }
    }
}

/**
 * @brief 从已校验帧队列读取一个完整数据帧
 * @param data 数据存放位置
 * @param length 读取长度，必须为 WT901_FRAME_SIZE
 * @retval bool 是否读取成功
 */
bool WT901_CirRead(uint8_t* data, uint32_t length)
{
    uint16_t read_pos;
    if ((data == 0) || (length != WT901_FRAME_SIZE))
    {
        return false;
    }
    read_pos = g_wt901_cirbuf.head;
    for (uint32_t index = 0U; index < length; index++)
    {
        if (read_pos == g_wt901_cirbuf.tail)
        {
            return false;
        }
        data[index] = g_wt901_cirbuf.cirbuf[read_pos];
        read_pos = WT901_CirNext(read_pos);
    }
    g_wt901_cirbuf.head = read_pos;
    return true;
}

/**
 * @brief 解析一个已接收的 WT901 数据帧
 * @retval bool 是否成功解析到一帧
 */
bool WT901_AnalyzeData(void)
{
    if (!WT901_CirRead(s_wt901_raw_data, WT901_FRAME_SIZE))
    {
        return false;
    }
    switch (s_wt901_raw_data[1])
    {
    case WT901_DATA_ACCEL:
        g_wt901_accel.x = (float)WT901_ReadS16(&s_wt901_raw_data[2]) * (9.8f * 16.0f / 32768.0f);
        g_wt901_accel.y = (float)WT901_ReadS16(&s_wt901_raw_data[4]) * (9.8f * 16.0f / 32768.0f);
        g_wt901_accel.z = (float)WT901_ReadS16(&s_wt901_raw_data[6]) * (9.8f * 16.0f / 32768.0f);
        g_wt901_temperature = (int16_t)(WT901_ReadS16(&s_wt901_raw_data[8]) / 100);
        break;
    case WT901_DATA_GYRO:
        g_wt901_gyro.x = (float)WT901_ReadS16(&s_wt901_raw_data[2]) * (2000.0f / 32768.0f);
        g_wt901_gyro.y = (float)WT901_ReadS16(&s_wt901_raw_data[4]) * (2000.0f / 32768.0f);
        g_wt901_gyro.z = (float)WT901_ReadS16(&s_wt901_raw_data[6]) * (2000.0f / 32768.0f);
        break;
    case WT901_DATA_ANGLE:
        g_wt901_angle.roll = (float)WT901_ReadS16(&s_wt901_raw_data[2]) * (180.0f / 32768.0f);
        g_wt901_angle.pitch = (float)WT901_ReadS16(&s_wt901_raw_data[4]) * (180.0f / 32768.0f);
        g_wt901_angle.yaw = (float)WT901_ReadS16(&s_wt901_raw_data[6]) * (180.0f / 32768.0f);
        g_wt901_angle_update_count++;
        if (!s_wt901_version_obtained)
        {
            g_wt901_version = WT901_ReadS16(&s_wt901_raw_data[8]);
            s_wt901_version_obtained = true;
        }
        break;
    default:
        g_wt901_unknown_frame_count++;
        break;
    }
    g_wt901_last_frame_type = s_wt901_raw_data[1];
    g_wt901_parse_count++;
    return true;
}
