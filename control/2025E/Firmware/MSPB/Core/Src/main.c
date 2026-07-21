/**
 * @file        main.c
 * @author      JerryZheng
 * @brief       MSPB 云台主控程序入口
 * @date        2026-07-21
 */

#include "app_mspb.h"
#include "board.h"

/**
 * @brief 启动流程诊断标记，供调试器观察最后到达的初始化阶段
 */
volatile uint32_t g_mspb_boot_stage = 0U;
volatile uint32_t g_mspb_loop_count = 0U;
/**
 * @brief MSPB 主函数
 * @retval int 程序不会返回
 */
int main(void)
{
    g_mspb_boot_stage = 1U;
    board_init();

    g_mspb_boot_stage = 2U;
    APP_MSPB_Init();

    g_mspb_boot_stage = 3U;

    while (1)
    {
        APP_MSPB_Process();
        //g_mspb_loop_count++;
    }
}
