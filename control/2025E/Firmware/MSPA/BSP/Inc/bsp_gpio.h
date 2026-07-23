/**
 * @file        bsp_gpio.h
 * @author      JerryZheng
 * @brief       底盘通用 GPIO 硬件抽象层接口
 * @date        2026-07-22
 */

#pragma once

#include <stdbool.h>

/**
 * @brief 读取板载用户按键当前状态
 * @retval true PA18 为高电平，按键已按下
 * @retval false PA18 为低电平，按键未按下
 */
bool BSP_GPIO_IsUserButtonPressed(void);
