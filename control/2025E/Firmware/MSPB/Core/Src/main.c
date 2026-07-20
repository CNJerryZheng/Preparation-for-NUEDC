#include "board.h"

int main(void)
{
    board_init();
    while (1)
    {
        if ((DL_GPIO_readPins(BSP_PORT, BSP_KEY_PIN) & BSP_KEY_PIN) != 0U)
        {
            DL_GPIO_clearPins(MSP_PORT, MSP_LED_BLUE_PIN);
            DL_GPIO_setPins(MSP_PORT, MSP_LED_GREEN_PIN);
        }
        else
        {
            DL_GPIO_setPins(MSP_PORT, MSP_LED_BLUE_PIN);
            DL_GPIO_clearPins(MSP_PORT, MSP_LED_GREEN_PIN);
        }
    }
}
