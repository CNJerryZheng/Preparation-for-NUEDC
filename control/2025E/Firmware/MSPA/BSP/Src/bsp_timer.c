/**
 * @file        bsp_timer.c
 * @author      JerryZheng
 * @brief       MSPA 底盘控制节拍 BSP 实现
 * @date        2026-07-21
 */

#include "bsp_timer.h"
#include "ti_msp_dl_config.h"

static volatile bool s_chassis_tick = false;

/**
 * @brief 使能 TIMG6 的 10ms 控制节拍中断
 */
void BSP_Timer_Init(void)
{
    NVIC_EnableIRQ(TIMER_CHASSIS_10MS_INST_INT_IRQN);
}

/**
 * @brief 取走一次待处理的 10ms 节拍
 * @retval bool 是否有新的 10ms 节拍
 */
bool BSP_Timer_TakeChassisTick(void)
{
    bool tick = s_chassis_tick;
    s_chassis_tick = false;
    return tick;
}

/**
 * @brief TIMG6 零点中断服务函数
 */
void TIMG6_IRQHandler(void)
{
    if (DL_Timer_getPendingInterrupt(TIMER_CHASSIS_10MS_INST) == DL_TIMER_IIDX_ZERO)
    {
        DL_Timer_clearInterruptStatus(TIMER_CHASSIS_10MS_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
        s_chassis_tick = true;
    }
}
