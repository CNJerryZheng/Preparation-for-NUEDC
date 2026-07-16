/**
 * @file        button.h
 * @author      JerryZheng
 * @brief       按钮驱动
 * @warning     按钮为高电平触发
 * @date        2026-07-14
*/

#pragma once

#include <main.h>
#include <stdbool.h>

/* <---------------------配置相关---------------------> */
#define BUTTON_DEBOUNCE_TIME 20U

/* <---------------------函数相关---------------------> */
/**
 * @brief 重置按钮状态
 */
void Button_Reset(void);

/**
 * @brief 扫描按钮状态
 *
 * @retval bool 触发返回1，没触发返回0
 */
bool Button_Scan(void);