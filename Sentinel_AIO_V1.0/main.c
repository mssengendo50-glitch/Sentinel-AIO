#include "ti_msp_dl_config.h"
#include "HAL/i2c.h"
#include "functions.h"
#include "HAL/uart.h"
#include "ics/BQ25628/BQ25628_functions.h"
#include "ics/BQ27Z7/BQ27Z7_functions.h"
#include "HAL/spi_master.h"
#include "HAL/ticks.h"
#include "sm.h"
#include "helper_functions.h"
#include "ics/ZILOG/ZDP323B.h"
#include "ics/LTR329/LTR329.h"
#include "ics/LIS3DH/LIS3DH.h"

volatile bool bq_monitor_active    = false;
volatile bool hall_monitor_active  = false;
volatile bool gauge_monitor_active = false;
volatile bool pir_monitor_active   = false;
volatile bool ltr_monitor_active   = false;
volatile bool ltr_model_monitor_active = false;
volatile bool lis_monitor_active   = false;
volatile bool rtc_minute_tick  = false;
volatile bool rtc_second_tick  = false;
volatile bool hall_wakeup_flag = false;
/* COUNTER, NOT A FLAG - AND THAT DISTINCTION IS THE WHOLE PROTOCOL.
 *
 * The STM32 raises IO2 once per transfer it wants clocked, and a request needs
 * TWO: one for the leg that carries the request up, one for the leg that
 * carries the reply back. As a bool, two edges arriving before the main loop
 * next looked collapsed into one - so we clocked the request leg, staged the
 * reply, and then sat waiting for a toggle the STM32 had already sent.
 *
 * Measured from the STM32 at the moment it gave up on /mspm0:
 *
 *     SPI5 at timeout: State=0x05 ErrorCode=0x00000000 rx_left=512 tx_left=496
 *
 * rx_left == 512 is the proof: not one byte was ever shifted in. Its DMA was
 * armed and correct (tx_left 496 is just the 16-byte TX FIFO pre-fill, which
 * happens without any clock) and no error was flagged. The reply transfer
 * simply never happened, because its toggle had been swallowed.
 *
 * Counting the edges instead means each one is serviced as its own transfer.
 * Saturates rather than wraps: if it ever runs away, losing edges at the top is
 * far better than wrapping to zero and losing all of them. */
volatile uint8_t stm_io2_edges = 0U;
/* When the most recent IO2 edge arrived, in microseconds since the STM32 rail
 * came up. Stamped in the interrupt, read in POWER_STM.
 *
 * This exists to measure the one deadline in this design that is not ours to
 * negotiate: FSBL arms its SPI slave, toggles IO2, and blocks for
 * AE_SEED_TIMEOUT_MS (60 ms). If the main loop does not get round to arming
 * inside that window the seed is lost and the STM32 falls back to a ~1 s blind
 * start. The gap between this stamp and the arm is that margin, measured
 * rather than assumed - and on a 9600-baud blocking console, a single debug
 * line is 40-60 ms of it.
 *
 * Reads Ticks_us() from interrupt context, which is safe: it only reads. If
 * SysTick cannot preempt this handler the value can understate by up to one
 * millisecond, which is immaterial against a 60 ms budget. */
volatile uint32_t stm_io2_edge_us = 0U;

/* CAM_SYNC edge count. The STM32's FSBL toggles this line immediately after its
 * frame-wait loop finishes - "the picture is taken" - and gpio.c drives it low
 * at every FSBL boot, so each power-on yields one clean low->high edge.
 *
 * Counted rather than flagged, for the same reason stm_io2_edges is: a bool
 * loses a second edge arriving before the main loop next looks. Here that would
 * leave a 900 mA illuminator burning until the timeout caught it. Saturates
 * rather than wraps. */
volatile uint8_t cam_sync_edges = 0U;

/* Wake trigger identity, latched in the interrupt that started the clock.
 * 0 = none, 1 = PIR, 2 = hall/setup. Read by the timing report so the number
 * can be attributed - a PIR wake and a magnet wake have very different
 * budgets and should never be averaged together. */
volatile uint8_t wake_trigger_src = 0U;
volatile uint32_t monitor_rate = 200; 
volatile uint32_t EEPROMEmulationState;  

void setupCLI(void) {
    CLI_RegisterCommand("help", cmd_help, "Show available commands");
    CLI_RegisterCommand("pwr",  cmd_pwr,  "Control power rails: 3v8, lora, lte, wifi, stm");
    CLI_RegisterCommand("i2cscan", cmd_i2cscan, "Scan I2C bus: i2cscan <0|1>");
    CLI_RegisterCommand("i2cscan10", cmd_i2cscan10, "Scan 10bit I2C bus: i2cscan10 <0|1>");
    CLI_RegisterCommand("hall", cmd_hall, "Hall sensor: hall <pwr|status>");
    CLI_RegisterCommand("bq", cmd_bq, "BQ25628E charger control - type bq for full help");
    CLI_RegisterCommand("gauge",   cmd_gauge,   "BQ27Z746 gauge - type gauge for help");
    CLI_RegisterCommand("sm", cmd_sm, "State Machine control: status, start, stop");
    CLI_RegisterCommand("led",    cmd_leds,    "LED control - type led for full help");
    CLI_RegisterCommand("pir",     cmd_pir,     "PIR monitor - type pir for full help");
    CLI_RegisterCommand("ltr",     cmd_ltr,     "LTR-329 ALS sensor - type ltr for help");
    CLI_RegisterCommand("lis",     cmd_lis,     "LIS3DH accelerometer - type lis for help");
    CLI_RegisterCommand("imx",     cmd_imx,     "IMX335 camera over I2C1 - type imx for help");
}


int main(void)
{
    SYSCFG_DL_init(); 
    /* Takes SysTick off SysConfig's generated 2-cycle reload and leaves it
     * stopped. SM_SetSTMPower() starts it when the STM32 rail comes up. */
    Ticks_Init();
    setupCLI();
    hall_init();
    gauge_init();
    PWR_EnableCoreInterrupts();
    char processingBuffer[MAX_INPUT_LEN];
    SM_Init();
    while (1) {
        if (data_received) {
            get_UART_buffer(processingBuffer);
            CLI_ProcessInput(processingBuffer);
        }
        if (!sm_context.sm_paused) {
            SM_Run();
        } else {
            Run_Legacy_Monitors(processingBuffer);
        }
    }
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            UARTReceive();
            break;
        default:
            break;
    }
}

void RTC_IRQHandler(void)
{
    switch (DL_RTC_getPendingInterrupt(RTC)) {
        case DL_RTC_IIDX_INTERVAL_TIMER:
            rtc_minute_tick = true;
            break;

        case DL_RTC_IIDX_PRESCALER1:
            rtc_second_tick = true;
            break;
        default:
            break;
    }
}


/*
 * GROUP1 carries more than just GPIOA and GPIOB.
 *
 * Bounded loop drains all pending group-1 sources and clears unhandled interrupt flags
 * to prevent interrupt storms that stall the main loop.
 */
void GROUP1_IRQHandler(void) {
    for (uint8_t guard = 0U; guard < 8U; guard++) {
        uint32_t pending = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

        if (pending == 0U) {
            break;                      /* nothing left to service */
        }

        switch (pending) {
            case EXTERNAL_INTERRUPT_GPIOA_INT_IIDX: 
                switch (DL_GPIO_getPendingInterrupt(GPIOA)) {
                    case EXTERNAL_INTERRUPT_PIR_TRIGGER_IIDX:
                        /* t = 0. FIRST STATEMENT, BEFORE ANYTHING ELSE.
                         *
                         * The requirement is 200 ms from here to a captured
                         * image, so this instant is the origin every other
                         * number in the system is quoted against. Starting the
                         * clock before the housekeeping below - rather than
                         * after, or in the main loop - means the measurement
                         * includes the housekeeping instead of hiding it.
                         *
                         * Coming out of STANDBY0 the core was not running until
                         * this edge, so this is the earliest observable instant;
                         * the wake transition itself is microseconds and is the
                         * one part of the budget this cannot account for. */
                        Ticks_Start();
                        wake_trigger_src = 1U;   /* PIR */

                        DL_GPIO_clearInterruptStatus(GPIOA, EXTERNAL_INTERRUPT_PIR_TRIGGER_PIN);
                        pir_monitor_active = true;
                        ZDP323B_MotionISR();
                        PIR_interrupt(false);
                        break;
                    case EXTERNAL_INTERRUPT_STM_MCU_IO2_IIDX:
                        DL_GPIO_clearInterruptStatus(GPIOA, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
                        stm_io2_edge_us = Ticks_us();
                        if (stm_io2_edges < 255U) {
                            stm_io2_edges++;
                        }
                        break;

                    default:
                        DL_GPIO_clearInterruptStatus(GPIOA, 0xFFFFFFFFU);
                        break;
                }
                break;

            case EXTERNAL_INTERRUPT_GPIOB_INT_IIDX: 
                switch (DL_GPIO_getPendingInterrupt(GPIOB)) {
                    case EXTERNAL_INTERRUPT_SETUP_INT_IIDX:
                        /* Same origin treatment as the PIR. A magnet wake is not on the
                         * 200 ms clock - an operator is standing there - but measuring
                         * it the same way costs nothing and keeps one code path. */
                        Ticks_Start();
                        wake_trigger_src = 2U;   /* hall / setup */

                        DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
                        hall_wakeup_flag = true;
                        break;

                    case EXTERNAL_INTERRUPT_CAM_SYNC_IIDX:
                        /* The STM32 has finished capturing - FSBL toggles this
                         * straight after its frame-wait loop on PB10. Emphatically does
                         * NOT call Ticks_Start(): this lands in the middle of
                         * the window that clock is measuring, and restarting it
                         * here would rebase every stage timing for the wake. */
                        DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_CAM_SYNC_PIN);
                        if (cam_sync_edges < 255U) {
                            cam_sync_edges++;
                        }
                        break;

                    default:
                        DL_GPIO_clearInterruptStatus(GPIOB, 0xFFFFFFFFU);
                        break;
                }
                break;

            default:
                DL_GPIO_clearInterruptStatus(GPIOA, 0xFFFFFFFFU);
                DL_GPIO_clearInterruptStatus(GPIOB, 0xFFFFFFFFU);
                break;
        }
    }
}
