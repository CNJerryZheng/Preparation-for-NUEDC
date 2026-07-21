/**
 * @file        main.c
 * @author      JerryZheng
 * @brief       MSPA 底盘主控程序入口
 * @date        2026-07-21
 */

#include "board.h"
#include "app_mspa.h"

/**
 * @brief MSPA 主函数
 * @retval int 程序不会返回
 */
int main(void)
{
    board_init();
    APP_MSPA_Init();
    while (1)
    {
        APP_MSPA_Process();
    }
}
