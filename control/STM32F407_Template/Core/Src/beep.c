/**
 * @file        beep.c
 * @author      JerryZheng
 * @brief       Active-low buzzer driver
 * @date        2026-07-14
 */

#include "beep.h"

typedef enum
{
    BEEP_STATE_IDLE = 0,
    BEEP_STATE_ON,
    BEEP_STATE_OFF,
} BEEP_StateTypeDef;

static BEEP_StateTypeDef s_beep_state = BEEP_STATE_IDLE;
static uint8_t s_beep_remaining;
static uint32_t s_beep_on_ms;
static uint32_t s_beep_off_ms;
static uint32_t s_beep_tick;

static void BEEP_On(void)
{
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
}

static void BEEP_Off(void)
{
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
}

void BEEP_Init(void)
{
    BEEP_Stop();
}

void BEEP_Stop(void)
{
    BEEP_Off();
    s_beep_state = BEEP_STATE_IDLE;
    s_beep_remaining = 0U;
}

bool BEEP_IsBusy(void)
{
    return s_beep_state != BEEP_STATE_IDLE;
}

void BEEP_Play(uint8_t count, uint32_t on_ms, uint32_t off_ms)
{
    if ((count == 0U) || (on_ms == 0U))
    {
        BEEP_Stop();
        return;
    }

    s_beep_remaining = count;
    s_beep_on_ms = on_ms;
    s_beep_off_ms = off_ms;
    s_beep_tick = HAL_GetTick();
    s_beep_state = BEEP_STATE_ON;
    BEEP_On();
}

void BEEP_Fast(uint8_t count)
{
    BEEP_Play(count, BEEP_FAST_ON_MS, BEEP_FAST_OFF_MS);
}

void BEEP_Slow(uint8_t count)
{
    BEEP_Play(count, BEEP_SLOW_ON_MS, BEEP_SLOW_OFF_MS);
}

void BEEP_Once(void)
{
    BEEP_Fast(1U);
}

void BEEP_Update(void)
{
    uint32_t now;

    if (s_beep_state == BEEP_STATE_IDLE)
    {
        return;
    }

    now = HAL_GetTick();
    if ((s_beep_state == BEEP_STATE_ON) &&
        ((now - s_beep_tick) >= s_beep_on_ms))
    {
        BEEP_Off();
        if (s_beep_remaining <= 1U)
        {
            s_beep_state = BEEP_STATE_IDLE;
            s_beep_remaining = 0U;
        }
        else
        {
            --s_beep_remaining;
            s_beep_state = BEEP_STATE_OFF;
            s_beep_tick = now;
        }
    }
    else if ((s_beep_state == BEEP_STATE_OFF) &&
             ((now - s_beep_tick) >= s_beep_off_ms))
    {
        s_beep_state = BEEP_STATE_ON;
        s_beep_tick = now;
        BEEP_On();
    }
}
