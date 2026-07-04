#include "board.h"

#define SYSTICK_COUNTER_MASK     (0x00FFFFFFUL)
#define CPU_TICKS_PER_US         (CPUCLK_FREQ / 1000000UL)
#define MAX_DELAY_US_PER_CHUNK   (SYSTICK_COUNTER_MASK / CPU_TICKS_PER_US)

#if (CPUCLK_FREQ % 1000000UL) != 0
#error "CPUCLK_FREQ must be an integer multiple of 1 MHz"
#endif

static void delay_cycles_systick(uint32_t cycles)
{
    const uint32_t start = SysTick->VAL;

    while (((start - SysTick->VAL) & SYSTICK_COUNTER_MASK) < cycles)
    {
        __NOP();
    }
}

void board_init(void)
{
    SYSCFG_DL_init();

    /* delay_us() 使用完整的 24 位回卷周期。 */
    DL_SYSTICK_init(SYSTICK_COUNTER_MASK + 1UL);
    DL_SYSTICK_enable();
}

void delay_us(uint32_t us)
{
    while (us > 0U)
    {
        const uint32_t chunk_us =
            (us > MAX_DELAY_US_PER_CHUNK) ? MAX_DELAY_US_PER_CHUNK : us;

        delay_cycles_systick(chunk_us * CPU_TICKS_PER_US);
        us -= chunk_us;
    }
}

void delay_ms(uint32_t ms)
{
    while (ms > 0U)
    {
        delay_us(1000U);
        --ms;
    }
}
