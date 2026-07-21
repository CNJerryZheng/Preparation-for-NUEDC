#include "board.h"
#include "app_mspb.h"

int main(void)
{
    board_init();
    APP_MSPB_Init();
    while (1)
    {
        APP_MSPB_Process();
    }
}
