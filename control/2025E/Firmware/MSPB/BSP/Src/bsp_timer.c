/**
 * @file        bsp_timer.c
 * @author      JerryZheng
 * @brief       MSPB 云台控制节拍 BSP 实现
 * @date        2026-07-21
 */

#include "bsp_timer.h"
#include "ti_msp_dl_config.h"

static volatile bool s_gimbal_tick = false;
volatile uint32_t g_gimbal_tick_count = 0U;

/**
 * @brief 使能 TIMG6 的 1ms 控制节拍中断
 */
void BSP_Timer_Init(void)
{
    NVIC_EnableIRQ(TIMER_GIMBAL_1MS_INST_INT_IRQN);
}

/**
 * @brief 取走一次待处理的 1ms 节拍
 * @retval bool 是否有新的 1ms 节拍
 */
bool BSP_Timer_TakeGimbalTick(void)
{
    bool tick = s_gimbal_tick;
    s_gimbal_tick = false;
    return tick;
}

/**
 * @brief TIMG6 零点中断服务函数
 */
void TIMG6_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(TIMER_GIMBAL_1MS_INST) == DL_TIMER_IIDX_ZERO)
    {
        DL_Timer_clearInterruptStatus(TIMER_GIMBAL_1MS_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
        s_gimbal_tick = true;
        g_gimbal_tick_count++;
    }
}
