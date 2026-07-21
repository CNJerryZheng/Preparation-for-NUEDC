/**
 * @file        wt901.c
 * @author      Misybon
 *             JerryZheng
 * @brief       WT901 communication driver.
 * @warning     The original F407 receive/parse flow is retained; only HAL DMA access is replaced by UART ISR input.
 * @date        2026-07-20
 */

#include "wt901.h"

struct WT901_Accel g_wt901_accel = { 0.0f, 0.0f, 0.0f };
struct WT901_Gyro g_wt901_gyro = { 0.0f, 0.0f, 0.0f };
struct WT901_Angle g_wt901_angle = { 0.0f, 0.0f, 0.0f };
int16_t g_wt901_temperature = 0;
int16_t g_wt901_version = 0;
volatile uint8_t g_wt901_buf[WT901_BUF_SIZE] = { 0U };
volatile WT901_CircularBuffer g_wt901_cirbuf = { { 0U }, 0U, 0U };
volatile uint32_t g_wt901_lose_count = 0U;
volatile uint32_t g_wt901_count = 0U;

static uint8_t s_wt901_raw_data[WT901_FRAME_SIZE] = { 0U };
static uint8_t s_wt901_framebuf[WT901_FRAME_SIZE] = { 0U };
static uint16_t s_frame_pos = 0U;
static bool s_wt901_version_obtained = false;

/**
 * @brief Check the native WT901 checksum (sum of bytes 0 through 9).
 */
static bool WT901_CheckSum(const uint8_t *data)
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

/** @brief Get the next position in the verified-frame ring buffer. */
static uint16_t WT901_CirNext(uint16_t index)
{
    return (uint16_t)((index + 1U) % WT901_CIR_SIZE);
}

/**
 * @brief Move a possible next frame header to the beginning after a malformed frame.
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

/** @brief Copy a verified frame into the task-consumed circular buffer. */
static bool WT901_CirWriteFrame(void)
{
    uint16_t write_pos;
    if (!WT901_CheckSum(s_wt901_framebuf))
    {
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

/** @brief Read a signed little-endian 16-bit payload value. */
static int16_t WT901_ReadS16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

void WT901_Init(void)
{
    g_wt901_cirbuf.head = 0U;
    g_wt901_cirbuf.tail = 0U;
    s_frame_pos = 0U;
    s_wt901_version_obtained = false;
}

uint16_t WT901_BufNext(uint16_t index)
{
    return (uint16_t)((index + 1U) % WT901_BUF_SIZE);
}

void WT901_CirWrite_Data(uint8_t data)
{
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

bool WT901_CirRead(uint8_t *data, uint32_t length)
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
        if (!s_wt901_version_obtained)
        {
            g_wt901_version = WT901_ReadS16(&s_wt901_raw_data[8]);
            s_wt901_version_obtained = true;
        }
        break;

    default:
        break;
    }
    return true;
}
