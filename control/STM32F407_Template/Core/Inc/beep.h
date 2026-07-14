/**
 * @file        beep.h
 * @author      JerryZheng
 * @brief       蜂鸣器驱动
 * @date        2026-07-14
 */

#pragma once

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BEEP_FAST_ON_MS 80U
#define BEEP_FAST_OFF_MS 80U
#define BEEP_SLOW_ON_MS 250U
#define BEEP_SLOW_OFF_MS 250U

void BEEP_Init(void);
void BEEP_Update(void);
void BEEP_Stop(void);
bool BEEP_IsBusy(void);
void BEEP_Play(uint8_t count, uint32_t on_ms, uint32_t off_ms);
void BEEP_Fast(uint8_t count);
void BEEP_Slow(uint8_t count);
void BEEP_Once(void);

#ifdef __cplusplus
}
#endif
