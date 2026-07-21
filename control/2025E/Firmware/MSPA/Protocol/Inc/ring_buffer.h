/**
 * @file        ring_buffer.h
 * @author      JerryZheng
 * @brief       Fixed-size byte ring buffer.
 * @date        2026-07-20
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t *data;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
} RING_Buffer_t;

/** @brief Initialize a ring buffer using caller-owned storage. */
void RING_BufferInit(RING_Buffer_t *buffer, uint8_t *data, uint16_t size);
/** @brief Write one byte; returns false when the buffer is full. */
bool RING_BufferWrite(RING_Buffer_t *buffer, uint8_t data);
/** @brief Read one byte; returns false when the buffer is empty. */
bool RING_BufferRead(RING_Buffer_t *buffer, uint8_t *data);
