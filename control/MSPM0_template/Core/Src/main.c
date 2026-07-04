#include "board.h"

int main(void)
{
    board_init();

    while (1)
    {
        DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_PIN_LED_PIN);
        delay_ms(1000);
    }
}
