#include "board.h"
#include "app_mspa.h"

int main(void)
{
    board_init();
    APP_MSPA_Init();
    while (1)
    {
        APP_MSPA_Process();
    }
}
