#ifndef TICKS_H
#define TICKS_H

#include <stdint.h>
#include <stdbool.h>

/* ── Microsecond time base for the STM32 power-on window ──────────────────
 *
 * WHAT THIS IS FOR, AND WHAT IT IS NOT
 *
 * This is not a wall clock. It is a stopwatch, and it reads zero at the WAKE
 * TRIGGER - the PIR edge, the hall edge, or the periodic-wake decision. Not at
 * rail-up, which is merely one of the stages it measures.
 *
 * The origin matters because the requirement does. The budget is "200 ms from
 * PIR to image captured", so the PIR edge is t = 0 by definition; anything
 * measured from the rail has already spent an unknown part of that budget
 * before its first sample. Rail-up is recorded as a stage instead - and it is
 * also the STM32's own t = 0, which is what lets the two timelines be joined.
 *
 * sm_context.second_counter remains the wall clock. It is driven by the RTC,
 * survives standby, and is what timeouts and schedules use. This is a different
 * instrument for a different question - do not substitute one for the other.
 *
 * WHY IT IS STOPPED THE REST OF THE TIME
 *
 * SysTick is clocked from the CPU and interrupts at 1 kHz. Leaving that running
 * would wake the core a thousand times a second out of the __WFI in IDLE and
 * CHARGING, which is where this product spends essentially all of its life -
 * a straight power regression on a battery device, paid for a number nobody
 * reads in those states.
 *
 * So the window is opened by the wake trigger - Ticks_Start() in the GROUP1
 * interrupt handler, or Ticks_StartIfIdle() on the periodic path that has no
 * edge - and closed by SM_SetSTMPower(false), the one function every rail-down
 * passes through, so a new exit path cannot forget to stop it.
 *
 * Starting it inside an interrupt is deliberate. Waking from STANDBY0 the core
 * is off until the edge arrives, so there is no earlier instant this firmware
 * can observe: t = 0 is the first instruction that runs because of the PIR. The
 * only thing it cannot see is the STANDBY0-to-RUN transition itself, which is
 * microseconds against a 200 ms budget.
 *
 * Two consequences worth knowing:
 *   - Reading this outside the window returns whatever the last window ended
 *     at. Ticks_Running() says whether the value means anything.
 *   - It cannot measure anything that spans standby, and it is not trying to.
 *
 * RESOLUTION
 *
 * 1 ms interrupt, with the SysTick counter itself read for the sub-millisecond
 * remainder - so 1 kHz of interrupt load buys microsecond resolution. On a
 * 32 MHz core the conversion is a shift, not a divide, which matters because
 * the M0+ has no hardware divider.
 *
 * ACCURACY CAVEAT: the count is in CPU cycles, so it is only microseconds while
 * the core is at 32 MHz. PWR_BlockFastClocks() drops the clock, and POWER_STM
 * calls PWR_EnterMeasureProfile() (which unblocks) on entry and stays there, so
 * inside the window this holds. Do not extend the window past that.
 */

/* Called once from main() after SYSCFG_DL_init(). Leaves SysTick STOPPED -
 * SysConfig's generated SYSCFG_DL_SYSTICK_init() sets a 2-cycle reload, which
 * is useless to us, and this takes the peripheral back off it. */
void Ticks_Init(void);

/* Zero and run. t = 0 is this instant. Safe to call from an interrupt.
 *
 * Calling it again while already running RESTARTS from zero - correct for a
 * fresh wake, wrong if a later stage calls it by mistake, which would silently
 * rebase every number after it. Ticks_StartIfIdle() is the defensive form for
 * code that only needs to guarantee a clock exists. */
void Ticks_Start(void);

/* Start only if not already running; returns true if it did. For paths that
 * may or may not have been entered from a wake interrupt - the periodic RTC
 * wake has no edge to stamp, so it starts its own clock, but must not restamp
 * over a PIR that arrived in the same pass. */
bool Ticks_StartIfIdle(void);

/* Stop and leave the interrupt disabled. The last value stays readable. */
void Ticks_Stop(void);

/* True between Start and Stop - i.e. whether a reading is meaningful. */
bool Ticks_Running(void);

/* Microseconds since Ticks_Start(). Wraps after ~71 minutes, far beyond the
 * ~600 s inactivity timeout that bounds any power-on window. */
uint32_t Ticks_us(void);

/* Milliseconds since Ticks_Start(). */
uint32_t Ticks_ms(void);

#endif /* TICKS_H */
