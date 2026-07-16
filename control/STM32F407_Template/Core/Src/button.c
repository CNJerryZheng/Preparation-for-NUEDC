/**
 * @file        button.c
 * @author      JerryZheng
 * @brief       按钮驱动
 * @warning     按钮为高电平触发
 * @date        2026-07-14
*/
#include "button.h"

/* <----------------数据变量----------------> */
static uint8_t button_armed = 1U;

/* <------------------函数------------------> */
void Button_Reset(void)
{
    button_armed = 1U;
}

bool Button_Scan(void)
{
    if (button_armed == 1U)
    {
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
        {
            HAL_Delay(BUTTON_DEBOUNCE_TIME);
            if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
            {
                button_armed = 0U;
                return true;
            }
        }
    }

    else
    {
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
        {
            HAL_Delay(BUTTON_DEBOUNCE_TIME);
            if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
            {
                button_armed = 1U;
            }
        }
    }

    return false;
}