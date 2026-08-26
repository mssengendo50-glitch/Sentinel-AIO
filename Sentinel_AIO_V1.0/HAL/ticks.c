#include "ticks.h"
#include "ti_msp_dl_config.h"

/* 32 MHz core. Both of these are exact powers-of-two-friendly on purpose:
 * TICKS_CYCLES_PER_US is 32, so the sub-millisecond conversion below is a
 * shift. If the core clock ever changes, change both and re-check that the
 * shift is still valid. */
#define TICKS_CPU_HZ            32000000UL
#define TICKS_CYCLES_PER_MS     (TICKS_CPU_HZ / 1000UL)   /* 32000 */
#define TICKS_US_SHIFT          5U                        /* /32    */

/* Written by the ISR, read by everything else. */
static volatile uint32_t g_ticks_ms;
static volatile bool     g_ticks_running;

/* Overrides the weak alias in startup_mspm0g350x_ticlang.c. */
void SysTick_Handler(void)
{
    g_ticks_ms++;
}

void Ticks_Init(void)
{
    SysTick->CTRL = 0U;          /* stop, and disable the interrupt      */
    SysTick->LOAD = TICKS_CYCLES_PER_MS - 1U;
    SysTick->VAL  = 0U;          /* also clears COUNTFLAG                */
    g_ticks_ms      = 0U;
    g_ticks_running = false;
}

void Ticks_Start(void)
{
    SysTick->CTRL = 0U;
    SysTick->LOAD = TICKS_CYCLES_PER_MS - 1U;
    SysTick->VAL  = 0U;
    g_ticks_ms    = 0U;
    /* CLKSOURCE (processor clock) | TICKINT | ENABLE */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
    g_ticks_running = true;
}

bool Ticks_StartIfIdle(void)
{
    if (g_ticks_running)
    {
        return false;
    }
    Ticks_Start();
    return true;
}

void Ticks_Stop(void)
{
    SysTick->CTRL = 0U;
    g_ticks_running = false;
}

bool Ticks_Running(void)
{
    return g_ticks_running;
}

uint32_t Ticks_us(void)
{
    uint32_t ms1, ms2, val;

    do {
        ms1 = g_ticks_ms;
        val = SysTick->VAL;
        ms2 = g_ticks_ms;
    } while (ms1 != ms2);

    /* VAL counts DOWN from LOAD, so elapsed-within-this-millisecond is the
     * complement. */
    return (ms1 * 1000U) + ((TICKS_CYCLES_PER_MS - val) >> TICKS_US_SHIFT);
}

uint32_t Ticks_ms(void)
{
    return g_ticks_ms;
}
