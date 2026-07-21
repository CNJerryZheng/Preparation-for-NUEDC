/**
 * @file        ring_buffer.c
 * @author      JerryZheng
 * @brief       Fixed-size byte ring buffer.
 * @date        2026-07-20
 */

#include "ring_buffer.h"

void RING_BufferInit(RING_Buffer_t *buffer, uint8_t *data, uint16_t size)
{
    buffer->data = data;
    buffer->size = size;
    buffer->head = 0U;
    buffer->tail = 0U;
}

bool RING_BufferWrite(RING_Buffer_t *buffer, uint8_t data)
{
    uint16_t next = (uint16_t)((buffer->tail + 1U) % buffer->size);
    if (next == buffer->head)
    {
        return false;
    }
    buffer->data[buffer->tail] = data;
    buffer->tail = next;
    return true;
}

bool RING_BufferRead(RING_Buffer_t *buffer, uint8_t *data)
{
    if (buffer->head == buffer->tail)
    {
        return false;
    }
    *data = buffer->data[buffer->head];
    buffer->head = (uint16_t)((buffer->head + 1U) % buffer->size);
    return true;
}
