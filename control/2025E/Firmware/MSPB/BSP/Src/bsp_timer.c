/**
 * @file        bsp_timer.c
 * @author      JerryZheng
 * @brief       MSPB 云台控制节拍 BSP 实现
 * @date        2026-07-21
 */

#include "bsp_timer.h"
#include "ti_msp_dl_config.h"

/** @brief 主循环上次读取到的1ms节拍累计值。 */
static uint32_t s_gimbal_last_taken_count = 0U;
/** @brief TIMG6产生的1ms节拍累计值，供Live Watch诊断。 */
volatile uint32_t g_gimbal_tick_count = 0U;

/**
 * @brief 使能 TIMG6 的 1ms 控制节拍中断
 */
void BSP_Timer_Init(void)
{
    NVIC_EnableIRQ(TIMER_GIMBAL_1MS_INST_INT_IRQN);
}

/**
 * @brief 读取距离上次调用累计经过的1ms节拍数
 * @return uint32_t 累计节拍数，0表示没有新节拍
 */
uint32_t BSP_Timer_TakeGimbalTicks(void)
{
    const uint32_t current_count = g_gimbal_tick_count;
    const uint32_t elapsed_ticks =
        current_count - s_gimbal_last_taken_count;

    s_gimbal_last_taken_count = current_count;
    return elapsed_ticks;
}

/**
 * @brief TIMG6 零点中断服务函数
 */
void TIMG6_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(TIMER_GIMBAL_1MS_INST) == DL_TIMER_IIDX_ZERO)
    {
        DL_Timer_clearInterruptStatus(TIMER_GIMBAL_1MS_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
        ++g_gimbal_tick_count;
    }
}
