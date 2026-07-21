/**
 * @file        wt901.h
 * @author      Misybon
 *             JerryZheng
 * @brief       WT901 communication driver.
 * @warning     Caller must not retain pointers to parser-internal buffers.
 * @date        2026-07-20
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "wt901_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief WT901 stream data type. */
typedef enum
{
    WT901_DATA_TIME = 0x50,
    WT901_DATA_ACCEL = 0x51,
    WT901_DATA_GYRO = 0x52,
    WT901_DATA_ANGLE = 0x53
} WT901_DataTypeDef;

/** @brief Acceleration in m/s^2. */
struct WT901_Accel
{
    float x;
    float y;
    float z;
};

/** @brief Angular velocity in degree/s. */
struct WT901_Gyro
{
    float x;
    float y;
    float z;
};

/** @brief Euler angle in degree. */
struct WT901_Angle
{
    float roll;
    float pitch;
    float yaw;
};

/** @brief Circular buffer used to transfer verified WT901 frames from ISR to task. */
typedef struct
{
    volatile uint8_t cirbuf[WT901_CIR_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} WT901_CircularBuffer;

extern struct WT901_Accel g_wt901_accel;
extern struct WT901_Gyro g_wt901_gyro;
extern struct WT901_Angle g_wt901_angle;
extern int16_t g_wt901_temperature;
extern int16_t g_wt901_version;
extern volatile uint8_t g_wt901_buf[WT901_BUF_SIZE];
extern volatile WT901_CircularBuffer g_wt901_cirbuf;
extern volatile uint32_t g_wt901_lose_count;
extern volatile uint32_t g_wt901_count;

/** @brief Reset parser state; it does not change WT901 serial configuration. */
void WT901_Init(void);
/** @brief Get the next index in the original DMA-sized buffer. */
uint16_t WT901_BufNext(uint16_t index);
/** @brief Read one verified 11-byte frame. */
bool WT901_CirRead(uint8_t *data, uint32_t length);
/** @brief Push one UART RX byte; safe to call from UART1 IRQ context. */
void WT901_CirWrite_Data(uint8_t data);
/** @brief Decode one queued frame into the exported WT901 data objects. */
bool WT901_AnalyzeData(void);

#ifdef __cplusplus
}
#endif
