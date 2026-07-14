/**
 * @file        beep.h
 * @author      JerryZheng
 * @brief       蜂鸣器驱动
 * @warning     蜂鸣器低电平触发
 * @date        2026-07-14
 */

#pragma once

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* <---------------------配置相关---------------------> */
#define BEEP_FAST_ON_MS 80U
#define BEEP_FAST_OFF_MS 80U
#define BEEP_SLOW_ON_MS 250U
#define BEEP_SLOW_OFF_MS 250U

/* <---------------------函数相关---------------------> */
/**
 * @brief 蜂鸣器初始化/重置状态
 */
void BEEP_Init(void);

/**
 * @brief 状态更新
 * 
 * @warning 需要在主循环持续调用更新否则会无法停止蜂鸣器
 */
void BEEP_Update(void);

/**
 * @brief 立即停止当前蜂鸣任务
 */
void BEEP_Stop(void);

/**
 * @brief 判断蜂鸣器当前有没有正在执行的任务
 * 
 * @retval bool true表示忙，false表示空闲
 */
bool BEEP_IsBusy(void);

/**
 * @brief 蜂鸣器响铃
 * 
 * @param count 响几次
 * @param on_ms 响的时间
 * @param off_ms 间隔时间
 */
void BEEP_Play(uint8_t count, uint32_t on_ms, uint32_t off_ms);

/**
 * @brief 快速蜂鸣
 * 
 * @param count 蜂鸣次数
 */
void BEEP_Fast(uint8_t count);

/**
 * @brief 慢速蜂鸣
 * 
 * @param count 蜂鸣次数
 */
void BEEP_Slow(uint8_t count);

/**
 * @brief 快速响一次
 */
void BEEP_Once(void);

#ifdef __cplusplus
}
#endif
