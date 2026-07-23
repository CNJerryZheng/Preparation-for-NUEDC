/**
 * @file        bsp_gpio.c
 * @author      JerryZheng
 * @brief       底盘通用 GPIO 硬件抽象层实现
 * @date        2026-07-22
 */

#include "bsp_gpio.h"
#include "ti_msp_dl_config.h"

/**
 * @brief 读取板载用户按键当前状态
 * @retval true PA18 为高电平，按键已按下
 * @retval false PA18 为低电平，按键未按下
 */
bool BSP_GPIO_IsUserButtonPressed(void)
{
    return (DL_GPIO_readPins(GPIO_USER_INPUT_PORT,
                GPIO_USER_INPUT_USER_KEY_PIN) &
            GPIO_USER_INPUT_USER_KEY_PIN) != 0U;
}
