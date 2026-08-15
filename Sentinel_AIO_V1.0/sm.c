#include "sm.h"
#include "ti_msp_dl_config.h"
#include "HAL/uart.h"
#include "HAL/i2c.h"
#include "HAL/spi_master.h"
#include "ics/BQ25628/BQ25628_functions.h"
#include "ics/BQ27Z7/BQ27Z7_functions.h"
#include <stdio.h>
#include <string.h>
#include "helper_functions.h"
#include "ics/ZILOG/ZDP323B.h"
#include "ics/LIS3DH/LIS3DH.h"
#include "ics/LTR329/LTR329.h"

/* ── PIR Configuration ───────────────────────────────────── */
#define SM_PIR_FILTER_STEP ZDP323B_FILTER_STEP_3
#define SM_PIR_FILTER_TYPE ZDP323B_FILTER_TYPE_C
#define SM_PIR_THRESHOLD   20

/* ── Timing Constants ────────────────────────────────────── */
#define SM_VBAT_LOW_MV            3000
#define SM_VBAT_FULL_MV           3650
#define SM_VBAT_CHARGE_START_MV   3400
#define SM_SAFETY_POLL_S          5
#define SM_INACTIVITY_TIMEOUT_S   600
#define SM_I2C_RETRY_S            5
#define SM_FAULT_RETRY_S          5
#define SM_LIFELINE_TIMEOUT_MINUTES 1440

/* ── Global Context ──────────────────────────────────────── */
SM_Context_t sm_context;
extern volatile bool rtc_minute_tick;
extern volatile bool rtc_second_tick;
extern volatile bool hall_wakeup_flag;
extern volatile uint8_t stm_io2_edges;   /* see the note on the definition in main.c */
extern volatile bool pir_monitor_active;
extern SPI_Controller_Handle stm32Spi;

/* ── Static variables ────────────────────────────────────── */
static uint32_t last_safety_status = 0;
static uint8_t last_charger_status = 0;
static char json_buf[600];

/* ── Internal Prototypes ─────────────────────────────────── */
static void SM_Handle_RTC_Tick(void);
static void SM_DecodeBatterySafetyStatus(uint32_t status);
static void SM_DecodeChargingSafetyStatus(uint8_t status);
static bool SM_NeedsPeriodicSTMWake(SM_PowerContext_t pwr);
static void SM_ResumeSystemContext(void);
static SM_PowerContext_t SM_FetchPowerContext(void);
static void SM_SetSTMPower(bool enable);
static void SM_SendOffer(void);
static void SM_DispatchIncomingPacket(void);
static bool SM_ProcessFault(uint32_t gauge_safety, uint8_t charger_fault);
static void SM_PrepareTelemetryResponse(void);
static void SM_PrepareSTMConfigResponse(void);
static void SM_PrepareSTMCredentialsResponse(void);
static bool SM_CheckExternalWakeTriggers(void);

static void SM_Heartbeat(void);

static void SM_HandleState_INIT(void);
static void SM_HandleState_CHARGING(void);
static void SM_HandleState_POWER_STM(void);
static void SM_HandleState_IDLE(void);
static void SM_HandleState_CRITICAL_FAULT(void);


/* ── Heartbeat ───────────────────────────────────────────── */

/*
 * Printed once per minute, on the minute-tick branch of IDLE and CHARGING —
 * i.e. immediately after PWR_EnterMeasureProfile() has brought the UART back
 * up, which is the only window in those states where a print is possible.
 *
 * Its real job is liveness: if these stop arriving, the main loop has stalled,
 * and the timestamp of the last one tells you when.
 */
static void SM_Heartbeat(void)
{
    uint32_t period      = sm_context.stm_wake_period.wake_interval_minutes;
    uint32_t since_wake  = sm_context.minute_counter - sm_context.last_stm_periodic_minute;
    uint32_t next_wake   = (since_wake >= period) ? 0U : (period - since_wake);

    uart_printf("[HB] %s | up %lum | %lum since STM wake | next in %lum | wakes:%lu | i2c t/o:%lu rec:%lu\n",
        SM_GetStateString(),
        (unsigned long)sm_context.minute_counter,
        (unsigned long)since_wake,
        (unsigned long)next_wake,
        (unsigned long)sm_context.total_wakes,
        (unsigned long)I2C_GetTimeoutCount(),
        (unsigned long)I2C_GetRecoveryCount());
}

/* ── Hardware Abstraction Helpers ────────────────────────── */

// Replaces repetitive GPIO calls for STM32 power
static void SM_SetSTMPower(bool enable) {
    if (enable) {
        PWR_EnterActiveProfile();
        DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN | DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
    } else {
        LTR329_SetMode(false);
        DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN | DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
        DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
        PWR_ExitActiveProfile();
    }
}

// Single source of truth for current power state
static SM_PowerContext_t SM_FetchPowerContext(void) {
    SM_PowerContext_t ctx;
    BQ25628E_ADC_Control(true);
    BQ27Z746_UpdateTelemetry(I2C_0_INST);
    BQ25628E_UpdateTelemetry();   
    uint16_t batt_status = BQ27Z746_Get_BatteryStatus();
    ctx.vbus_mv = BQ25628E_Get_VBUS_mV();
    ctx.vbat_mv = BQ27Z746_Get_Voltage_mV();
    ctx.stat1 = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
    ctx.adapter_present = (ctx.stat1 & BQ25628E_VBUS_STAT_MASK) != 0;
    ctx.chg_stat = (ctx.stat1 >> 3) & 0x03;
    ctx.is_critical_low = (ctx.vbat_mv < SM_VBAT_LOW_MV) && !ctx.adapter_present;
    bool gauge_charging = (batt_status & BQ27Z746_STATUS_DSG) == 0;
    bool charger_active = (ctx.chg_stat == 1 || ctx.chg_stat == 2);   
    ctx.is_charging = gauge_charging || charger_active;
    ctx.charger_done = (ctx.chg_stat == 0 || ctx.chg_stat == 3);
    BQ25628E_ADC_Control(false);
    return ctx;
}

/* ── Core State Machine Functions ────────────────────────── */

void SM_Init(void)
{
    memset(&sm_context, 0, sizeof(SM_Context_t));
    sm_context.current = SM_STATE_INIT;
    sm_context.first_boot = true;
    sm_context.last_lifeline_reset_minute = 0;
    SM_LoadCharger();
    SM_LoadPeriod();
    SM_LoadCredentials();
    SM_LoadSTMConfig();
    PIR_interrupt(false);
    pir_monitor_active = false;
}

void SM_Transition(SM_State_t new_state) {
    const char* old_name = SM_GetStateString();
    sm_context.previous = sm_context.current;
    sm_context.current = new_state;
    sm_context.entry_done = false;
    uart_printf("[SM] %s -> %s\n", old_name, SM_GetStateString());
}

const char* SM_GetStateString(void) {
    switch (sm_context.current) {
        case SM_STATE_INIT:           return "INIT";
        case SM_STATE_CHARGING:       return "CHARGING";
        case SM_STATE_POWER_STM:      return "POWER_STM";
        case SM_STATE_IDLE:           return "IDLE";
        case SM_STATE_CRITICAL_FAULT: return "CRITICAL_FAULT";
        default:                      return "UNKNOWN";
    }
}

const char* SM_GetChargeString(uint8_t chg_stat) {
    switch (chg_stat) {
        case 0: return "Not Charging / Terminated";
        case 1: return "Pre/Trickle/Fast (CC)"; 
        case 2: return "Taper (CV)";
        case 3: return "Top-Off";
        default: return "Unknown";
    }
}

SM_State_t SM_GetState(void) {
    return sm_context.current;
}

static void SM_PostWake_Branch(void) {
    SM_PowerContext_t pwr = SM_FetchPowerContext();
    if (pwr.is_critical_low) {
        uart_printf("[SM] Critical low battery detected, entering IDLE to conserve power\n");
        SM_Transition(SM_STATE_IDLE);      
    } else if (pwr.vbat_mv < SM_VBAT_CHARGE_START_MV && pwr.adapter_present) {
        SM_Transition(SM_STATE_CHARGING);
    } else {
        SM_Transition(SM_STATE_IDLE);   
    }
}

static void SM_Handle_RTC_Tick(void) {
    if (!rtc_minute_tick && !rtc_second_tick) return;
    
    if (rtc_minute_tick) {
        rtc_minute_tick = false;
        sm_context.minute_counter++;
    }
    if (rtc_second_tick) {
        rtc_second_tick = false;
        sm_context.second_counter++;
        if ((sm_context.second_counter - sm_context.last_safety_check_s) >= SM_SAFETY_POLL_S) {
            sm_context.last_safety_check_s = sm_context.second_counter;
            if (SM_SafetyCheck()) return;
        }
    }  
}

static bool SM_CheckExternalWakeTriggers(void) {
    if (hall_wakeup_flag) {
        hall_wakeup_flag = false;
        sm_context.wake_reason = SM_WAKE_SETUP;
        RTC_EnablePrescaler();
        PWR_EnterMeasureProfile();
        SM_Transition(SM_STATE_POWER_STM);
        return true;
    }
    if (pir_monitor_active) {
        pir_monitor_active = false;
        if (sm_context.stm_wake_period.wake_mode == 1) {
            sm_context.wake_reason = SM_WAKE_PIR;
            sm_context.last_lifeline_reset_minute = sm_context.minute_counter;
            RTC_EnablePrescaler();
            PWR_EnterMeasureProfile();
            SM_Transition(SM_STATE_POWER_STM);
            return true;
        }
    }
    return false;
}

static void SM_HandleState_INIT(void) {

    if (!sm_context.entry_done) {
        PWR_EnterMeasureProfile();
        
        if (!I2C_TryAddress(I2C_0_INST, GAUGE_I2C_ADDR) || !I2C_TryAddress(I2C_0_INST, BQ25628E_I2C_ADDR)) {
            sm_context.fault_source = SM_FAULT_I2C_BUS;
            SM_Transition(SM_STATE_CRITICAL_FAULT);
            return;
        }
        
        uint16_t pir_addr = ZDP_ScanAddresses(I2C_0_INST);
        if (pir_addr == 0) {
            uart_printf("[SM] PIR sensor not found on bus\n");
            sm_context.fault_source = SM_FAULT_I2C_BUS;
            SM_Transition(SM_STATE_CRITICAL_FAULT);
            return;
        }

        bool gauge_ok   = BQ27Z746_Init(I2C_0_INST);
        BQ25628E_HardwareInit();
        BQ25628E_ApplyProfile(&sm_context.sm_charger_config);
        bool charger_ok = true;
        
        I2C_Status pir_status = ZDP323B_Init(I2C_0_INST, pir_addr, SM_PIR_FILTER_STEP, SM_PIR_FILTER_TYPE, SM_PIR_THRESHOLD);
        bool pir_ok = (pir_status == I2C_SUCCESS);

        if (I2C_TryAddress(I2C_0_INST, LTR329_I2C_ADDR)) {
            LTR329_Init(I2C_0_INST);
        } else {
            uart_printf("[SM] ALS sensor not found\n");
        }

        uint8_t accel_addr = 0;
        if (I2C_TryAddress(I2C_0_INST, LIS3DH_I2C_ADDR_0)) accel_addr = LIS3DH_I2C_ADDR_0;
        else if (I2C_TryAddress(I2C_0_INST, LIS3DH_I2C_ADDR_1)) accel_addr = LIS3DH_I2C_ADDR_1;
        if (accel_addr != 0) {
            LIS3DH_Init(I2C_0_INST, accel_addr);
        } else {
            uart_printf("[SM] Accelerometer not found\n");
        }
                        
        if (!gauge_ok || !charger_ok || !pir_ok) {
            sm_context.fault_source = SM_FAULT_INIT_FAILED;
            SM_Transition(SM_STATE_CRITICAL_FAULT);
        } else {
            sm_context.entry_done = true;
            SM_PostWake_Branch();
        }
    }
}

static void SM_HandleState_CHARGING(void) {

    if (!sm_context.entry_done) {
        DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
        uart_printf("[SM] Charging started\n");
        sm_context.critical_msg_sent = false;
        sm_context.entry_done = true;
        sm_context.last_charging_tick = sm_context.minute_counter; 
        RTC_DisablePrescaler();
        DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        DL_GPIO_enableInterrupt(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        sm_context.wake_reason = SM_WAKE_NORMAL;
        PWR_ExitMeasureProfile();
        if (sm_context.stm_wake_period.wake_mode == 1) {
            PIR_interrupt(true);
        }
    }
    if (SM_CheckExternalWakeTriggers()) return;
    if (sm_context.minute_counter != sm_context.last_charging_tick) {
        sm_context.last_charging_tick = sm_context.minute_counter; 
        PWR_EnterMeasureProfile();
        SM_Heartbeat();
        if (SM_SafetyCheck()) return;
        if (SM_ChargingSafetyCheck()) return;
        SM_PowerContext_t pwr = SM_FetchPowerContext();
        if (pwr.vbat_mv >= SM_VBAT_FULL_MV || pwr.charger_done) {
            DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
            uart_printf("[SM] Charging complete (VBAT %dmV >= %dmV) / Charging terminated\n", pwr.vbat_mv, SM_VBAT_FULL_MV);
            SM_Transition(SM_STATE_IDLE);
            return;
        } else {
            uart_printf("[SM] CHARGING | VBUS:%4dmV VBAT:%4dmV IBAT:%4dmA SOC:%3d%% TBAT:%3.1fC TDIE:%3dC CHG_STAT:%s\n",
                BQ25628E_Get_VBUS_mV(), pwr.vbat_mv, BQ27Z746_Get_Current_mA(),
                BQ27Z746_Get_SOC_pct(), BQ25628E_Get_TBAT_C(), BQ25628E_Get_TDIE_C(),
                SM_GetChargeString(pwr.chg_stat));
        }            
        if (SM_NeedsPeriodicSTMWake(pwr)) {
            RTC_EnablePrescaler();
            sm_context.last_stm_periodic_minute = sm_context.minute_counter;
            SM_Transition(SM_STATE_POWER_STM);
            return;
        }
    }
    PWR_ExitMeasureProfile();
    __WFI();
}

/* Cycle counts for the power-cycle delays. 32 MHz core. */
#define SM_MS_CYCLES(ms)          ((uint32_t)(ms) * 32000UL)

/* How long the rail stays down. Long enough for the STM32's supply to collapse
   far enough that its BootROM runs from cold - a brown-out that leaves the
   external NOR in octal DTR is precisely the failure this exists to avoid. */
#define SM_POWER_CYCLE_OFF_MS     500U

/* Breathing room after the acknowledgement lands, before the rail drops. */
#define SM_POWER_CYCLE_SETTLE_MS   50U

/**
 * Drop and restore the STM32's power rail, keeping the wake reason.
 *
 * Called only from SM_HandleState_POWER_STM, and only once the acknowledgement
 * for PID_POWER_CYCLE has physically gone out - see the two-phase flags in
 * sm.h. Cutting earlier kills the STM32 mid-transfer and it never learns the
 * request was accepted.
 */
static void SM_DoPowerCycle(void) {
    sm_context.power_cycle_pending = false;
    sm_context.power_cycle_armed   = false;

    uart_printf("[SM] Power-cycling STM32 (reason kept: %s)\n",
        (sm_context.wake_reason == SM_WAKE_SETUP) ? "SETUP" : "NORMAL");

    delay_cycles(SM_MS_CYCLES(SM_POWER_CYCLE_SETTLE_MS));

    SM_SetSTMPower(false);
    delay_cycles(SM_MS_CYCLES(SM_POWER_CYCLE_OFF_MS));

    /* Re-run the POWER_STM entry block on the next tick of the state machine.
     * Deliberately not a state transition: we are already in this state and
     * want its entry actions repeated, not a state change. */
    sm_context.entry_done = false;
}

static void SM_HandleState_POWER_STM(void) {

    if (!sm_context.entry_done) {
        /* PORTED FROM Insect_Intel_V1.0, WHERE THIS EXCHANGE WORKS.
         *
         * The two products' SM_HandleState_POWER_STM() are otherwise identical
         * - and HAL/spi_master.c is byte-for-byte identical, as are the
         * PWR_* profile helpers - so these lines are the whole of the drift
         * between a supervisor that answers the STM32's time request and one
         * that does not.
         *
         * 1. The power-cycle flags. Insect Intel's reason, verbatim: a request
         *    that was accepted and then overtaken by an inactivity timeout or a
         *    shutdown would still be pending next time the STM32 came up, and
         *    would fire on the first staged response - a spurious power cycle
         *    with no one asking for it. That has not bitten yet only because
         *    nothing on this product sends PID_POWER_CYCLE; the firmware update
         *    path is about to, so it would have become live exactly when it
         *    mattered most.
         *
         * 2. PWR_EnterMeasureProfile(). It is idempotent - it returns
         *    immediately when I2C0 and UART0 are already up - so this costs
         *    nothing on the paths that already called it (SM_CheckExternalWakeTriggers
         *    does, for hall and PIR wakes). It is not free on the paths that did
         *    not: PWR_UnblockFastClocks() lives inside it, and entering this
         *    state with the fast clocks still blocked slows the main loop, which
         *    is the difference between dispatching the STM32's request before its
         *    second toggle arrives and after.
         *
         *    It also fixes an ordering bug that is independent of any of this:
         *    LTR329_SetMode(true) below needs I2C0, and nothing above it
         *    guaranteed I2C0 was powered. Insect Intel does not have that
         *    problem because it brings the bus up first.
         *
         * Re-entry after SM_DoPowerCycle() clears entry_done rather than
         * transitioning, so this block runs again on a power cycle - which is
         * what makes clearing the flags here correct rather than merely tidy. */
        sm_context.power_cycle_pending = false;
        sm_context.power_cycle_armed   = false;

        PWR_EnterMeasureProfile();

        sm_context.total_wakes++;
        if (sm_context.wake_reason == SM_WAKE_SETUP) {
            DL_GPIO_setPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
        } else {
            DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
        }
        RTC_EnablePrescaler();
        SM_SetSTMPower(true);
        LTR329_SetMode(true);
        sm_context.stm_power_on_s = sm_context.second_counter;
        sm_context.last_io2_activity_s = sm_context.second_counter;
        uart_printf("[SM] STM32 powered : reason: %s\n",
            (sm_context.wake_reason == SM_WAKE_SETUP) ? "SETUP" : 
            (sm_context.wake_reason == SM_WAKE_PIR)   ? "PIR"   : "NORMAL");
        sm_context.entry_done = true;
        sm_context.stm_data_sent = false;

        /* A STAGED REPLY MUST NOT SURVIVE A POWER CYCLE.
         *
         * Everything else that describes "where we are in a conversation with
         * the STM32" is reset here - stm_data_sent above, stm_io2_edges below -
         * but has_pending_response was not, and it is the one that decides
         * which BRANCH the next IO2 edge takes.
         *
         * The STM32 only collects a staged reply if it asks for it. When it
         * gives up first (its Mspmo_busy timeout is 1000 ms and fires while we
         * are still between main-loop passes) the reply is never collected and
         * this flag is still set when the rail comes back. The next session's
         * FIRST toggle - which carries the STM32's time request - then takes
         * the has_pending_response branch below: we clock the stale reply out,
         * and because that branch leaves stm_data_sent false, the dispatch
         * block never runs. The request is shifted in and silently discarded.
         *
         * The STM32 then toggles again for its response leg, we treat THAT as a
         * fresh offer, and what gets printed and dispatched is the zero-filled
         * dummy buffer it transmits on that leg. Dispatch sees msg_type 0, hits
         * default:, calls SM_PrepareNack() - and sets this flag again. The
         * fault re-arms itself, which is why the STM32 has failed to read the
         * clock on every boot rather than intermittently.
         *
         * Clearing it here is the whole fix. Anything staged before the rail
         * dropped is addressed to a session that no longer exists. */
        sm_context.has_pending_response = false;

        DL_GPIO_disableInterrupt(EXTERNAL_INTERRUPT_CHARGER_INT_PORT, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        DL_GPIO_enableInterrupt(EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
        stm_io2_edges = 0U;
    }
    /* DISPATCH BEFORE ARMING. THE ORDER OF THESE TWO BLOCKS IS THE PROTOCOL.
     *
     * This block used to sit BELOW the IO2 block, and that is what made the
     * STM32's time request unreadable even after the stale-reply latch was
     * fixed. The observed sequence was:
     *
     *   [SM] IO2: sending offer      <- STM32's Tx_request. rxBuf gets 02 02.
     *   [SM] IO2: sending offer      <- STM32's Rx_response, microseconds later
     *    Received from STM32: 00 00 ...
     *
     * Two toggles arrive back to back because the STM32 posts its response leg
     * straight out of the DMA-complete interrupt - it does not wait for us, and
     * it has no way to know when we are ready. With the IO2 block first, the
     * second toggle ran before this block had ever executed:
     *
     *   - it cleared rxDone, destroying the completion flag for the transfer
     *     that had just delivered the request, and
     *   - SM_SendOffer() -> SPI_Controller_Arm() memsets rxBuf, destroying the
     *     request itself.
     *
     * So the real packet was overwritten before anyone looked at it, and what
     * finally got printed and dispatched was the zero-filled dummy buffer the
     * STM32 transmits on its response leg. Dispatch then saw msg_type 0, hit
     * default:, and staged a NACK - which is why the third toggle reported
     * "sending staged reply" to an STM32 that had already timed out and
     * aborted.
     *
     * Running dispatch first fixes it: the completed transfer is consumed
     * before anything can re-arm over it, and the staged reply is then armed by
     * the IO2 block in the SAME pass, which is what the STM32's already-armed
     * response DMA is waiting for.
     *
     * THE ORIGINAL VERSION OF THIS PARAGRAPH ARGUED THAT NOTHING COULD BE LOST
     * "because stm_io2_flag is a single bool, so two edges arriving before we
     * look coalesce into one". That was true, and it was the bug, not the
     * justification: coalescing is exactly how the reply leg's toggle went
     * missing and left the STM32 waiting on a transfer that never happened
     * (rx_left == 512 at its timeout). The edges are counted now - see
     * stm_io2_edges in main.c - so each is serviced as its own transfer, and
     * this ordering stands on its own merits rather than on that mistake.
     *
     * If this ever needs revisiting: the deeper asymmetry is that the STM32
     * cannot know when a reply is staged. MSG_OFFER means "nothing for you", so
     * the honest fix on that side is to treat an OFFER received on the response
     * leg as "not ready, retry" rather than as an answer. */
    if (sm_context.stm_data_sent && stm32Spi.rxDone) {
        stm32Spi.rxDone = false;
        sm_context.stm_data_sent = false;
        uart_printf("\n t=%lu Received from STM32: ",
            (unsigned long)sm_context.second_counter);
        for (int i = 0; i < 8; i++) {
            uart_printf("%02X ", stm32Spi.rxBuf[i]);
        }
        uart_printf("\n");

        SM_DispatchIncomingPacket();

        /* Stamped again on the way out: the gap between these two numbers is
           how long dispatch took to build and stage the reply, which is the one
           quantity nobody has measured. If it is 0 the supervisor is not the
           slow party and the STM32's timeout is looking at the wrong thing. */
        uart_printf(" t=%lu staged=%u\n",
            (unsigned long)sm_context.second_counter,
            (unsigned int)sm_context.has_pending_response);

        /* A DISPATCH CAN CHANGE STATE. STOP TOUCHING THE SPI IF IT DID.
         *
         * MSG_SHUTDOWN cuts the STM32's rail and calls SM_ResumeSystemContext(),
         * which transitions us out of POWER_STM - but SM_DispatchIncomingPacket()
         * only `return`s from itself, so without this guard execution falls
         * straight into the IO2 block below and, if a last toggle from the
         * dying STM32 is still pending, arms a full 512-byte transfer to a
         * board whose power has just been removed.
         *
         * That orphan is visible in the log as an "[SM] IO2: sending offer"
         * printed BETWEEN "POWER_STM -> CHARGING" and "Charging started" - a
         * transfer belonging to no session. It clocks 512 bytes against a dead
         * slave, and the FIFO/DMA residue it leaves is what puts the NEXT
         * session's first frame 4 bytes out:
         *
         *     Received from STM32:
         *     00 00 00 00 02 02 00 00 ...
         *
         * which the supervisor then reads as msg_type 0, NACKs, and the STM32
         * reports as "MSPM0 time resonse msg_type: 6".
         *
         * This became reachable when the dispatch block was moved above the IO2
         * block to fix the request being overwritten. That reorder is correct
         * and stays; this is the half of it that was missing. Any pending edges
         * are deliberately left counted - POWER_STM's entry action zeroes
         * stm_io2_edges on the next wake, which is the right owner. */
        if (sm_context.current != SM_STATE_POWER_STM) {
            return;
        }
     }

    /* Consume exactly ONE edge per pass, under a critical section.
     *
     * One per pass rather than draining them: each edge is a transfer the STM32
     * is waiting on, and the reply to the second cannot be staged until the
     * first has been dispatched. Servicing them one at a time in loop order is
     * what keeps request and reply in step.
     *
     * The critical section is not decoration - the read-decrement is not atomic
     * on this core, and the GROUP1 handler increments from an interrupt. Losing
     * an edge to that race is the exact failure this counter replaced. */
    uint8_t io2_pending;
    __disable_irq();
    io2_pending = stm_io2_edges;
    if (io2_pending > 0U) {
        stm_io2_edges--;
    }
    __enable_irq();

    if (io2_pending > 0U) {
        sm_context.last_io2_activity_s = sm_context.second_counter;
        stm32Spi.rxDone = false;

        /* Which branch a toggle takes is the whole protocol, and until now it
         * was invisible from the log. Keep this: if the STM32 ever reports a
         * failed round trip again, the first thing to know is whether its
         * opening toggle of a session was answered with a reply (wrong - a
         * reply is outstanding that should not be) or with an offer (right). */
        /* Timestamped, because the question this log has to answer is no longer
           "which branch" but "how long". The STM32 gives the whole two-transfer
           round trip 1000 ms; without a clock on this side there is no way to
           tell a supervisor that answered in 50 ms from one that took 2 s, and
           both present identically to it as a timeout. One-second resolution is
           coarse but it is the only tick this firmware keeps, and it is enough
           to separate those two cases. */
        /* A (prev=tx%u rx%u) field lived here, reporting whether the PREVIOUS
         * transfer's DMA completed. It was removed because it could not answer
         * the question it was added for, and a confounded instrument is worse
         * than none. Recorded so nobody adds it back expecting more:
         *
         *   - The dispatch block clears rxDone before this line ever runs, so
         *     any transfer that carried a request always reported rx0. Only the
         *     staged-reply transfers reported honestly.
         *   - It reports the previous transfer, so the one that matters - the
         *     telemetry staged reply - is only visible on the NEXT toggle. The
         *     STM32 stops toggling after it times out (its abort-path toggle
         *     was removed, correctly), so that toggle never comes and the
         *     outcome is never printed.
         *
         * What it did establish before being retired: the RTC staged reply
         * reports (prev=tx1 rx1) on the following toggle - it completes cleanly
         * end to end. That is the working control. The telemetry staged reply
         * has never been observed either way.
         *
         * To finish this, instrument the completion DIRECTLY - a flag set in
         * SPI_1_INST_IRQHandler and printed on the next loop pass - rather than
         * inferring it from state another block has already consumed. */
        uart_printf("[SM] t=%lu IO2: %s\n",
            (unsigned long)sm_context.second_counter,
            sm_context.has_pending_response ? "sending staged reply" : "sending offer");

        if (sm_context.has_pending_response) {
            SPI_Controller_Arm(&stm32Spi);
            sm_context.has_pending_response = false;

            /* If that staged reply is the acknowledgement for a power-cycle
             * request, it is now on the wire. SPI_Controller_Arm() has just
             * cleared rxDone, so rxDone going true again means this transfer
             * finished and the STM32 has the answer. Only then may the rail
             * drop. */
            if (sm_context.power_cycle_pending) {
                sm_context.power_cycle_armed = true;
            }
        } else {
            SM_SendOffer();
            sm_context.stm_data_sent = true;
        }
    }

    /* Deferred power cycle. Reached only once the acknowledgement transfer
     * armed above has completed, so the STM32 knows the cut is coming.
     *
     * This sits after the dispatch block on purpose: that block only consumes
     * rxDone when stm_data_sent is set, which it is not for a staged response,
     * so the flag survives for us to test here. */
    if (sm_context.power_cycle_armed && stm32Spi.rxDone) {
        stm32Spi.rxDone = false;
        SM_DoPowerCycle();
        return;
    }

    /* Inactivity timeout — resets each time IO2 fires */
    if ((sm_context.second_counter - sm_context.last_io2_activity_s) >= SM_INACTIVITY_TIMEOUT_S) {
        sm_context.inactivity_timeouts++;
        uart_printf("[SM] STM32 inactivity timeout\n");
        SM_SetSTMPower(false);
        SM_ResumeSystemContext();
    }
}

static void SM_HandleState_IDLE(void) {

    if (!sm_context.entry_done) {
        sm_context.wake_reason = SM_WAKE_NORMAL;
        SM_SetSTMPower(false);
        sm_context.sleep_entry_minute = sm_context.minute_counter;
        DL_GPIO_disableInterrupt(EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
        DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        DL_GPIO_enableInterrupt(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        hall_wakeup_flag = false;
        pir_monitor_active = false;
        RTC_DisablePrescaler();
        SM_PowerContext_t pwr = SM_FetchPowerContext();
        if (pwr.vbat_mv < SM_VBAT_CHARGE_START_MV) {
            DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
            uart_printf("[SM] Charging in IDLE enabled\n");
        } else {
            DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
            uart_printf("[SM] Charging in IDLE disabled\n");
        }      
        uart_printf("[SM] Entering IDLE\n");
        if (sm_context.stm_wake_period.wake_mode == 1) {
            PIR_interrupt(true);
        }
        sm_context.entry_done = true;
        PWR_ExitMeasureProfile();
    }
    if (sm_context.sleep_entry_minute != sm_context.minute_counter) {
        sm_context.sleep_entry_minute = sm_context.minute_counter;
        PWR_EnterMeasureProfile();
        SM_Heartbeat();
        if (SM_SafetyCheck()) return;
        SM_PowerContext_t pwr = SM_FetchPowerContext();
        if (pwr.is_charging) {
            SM_Transition(SM_STATE_CHARGING);
            return;
        }
        if (SM_NeedsPeriodicSTMWake(pwr)) {
            RTC_EnablePrescaler();
            sm_context.last_stm_periodic_minute = sm_context.minute_counter;
            SM_Transition(SM_STATE_POWER_STM);
            return;
        }
    }
    if (SM_CheckExternalWakeTriggers()) return;
    PWR_ExitMeasureProfile();
    __WFI();
}

static void SM_HandleState_CRITICAL_FAULT(void) {
    if (!sm_context.entry_done) {
        SM_SetSTMPower(false);
        DL_GPIO_disableInterrupt(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
        RTC_EnablePrescaler();
        const char* fault_str = "UNKNOWN";
        switch (sm_context.fault_source) {
            case SM_FAULT_I2C_BUS:      fault_str = "I2C_BUS"; break;
            case SM_FAULT_GAUGE:        fault_str = "GAUGE"; break;
            case SM_FAULT_CHARGER:      fault_str = "CHARGER"; break;
            case SM_FAULT_INIT_FAILED:  fault_str = "INIT_FAILED"; break;
            default: break;
        }
        uart_printf("[SM] CRITICAL FAULT, source: %s\n", fault_str);
        sm_context.fault_retry_s = sm_context.second_counter;

        if (sm_context.fault_source == SM_FAULT_GAUGE) {
            SM_DecodeBatterySafetyStatus(last_safety_status);
        } else if (sm_context.fault_source == SM_FAULT_CHARGER) {
            SM_DecodeChargingSafetyStatus(last_charger_status);
        } else if (sm_context.fault_source == SM_FAULT_I2C_BUS) {
            uart_printf("[SM] I2C bus fault. Retrying every %ds\n", SM_I2C_RETRY_S);
        } else if (sm_context.fault_source == SM_FAULT_INIT_FAILED) {
            uart_printf("[SM] Initialization failed, power cycle board\n");
        }
        sm_context.entry_done = true;
    }

    /* INIT_FAILED is unrecoverable, nothing to retry */
    if (sm_context.fault_source == SM_FAULT_INIT_FAILED) return;

    uint32_t elapsed = sm_context.second_counter - sm_context.fault_retry_s;
    uint32_t interval = (sm_context.fault_source == SM_FAULT_I2C_BUS) ? SM_I2C_RETRY_S : SM_FAULT_RETRY_S;

    if (elapsed >= interval) {
        sm_context.fault_retry_s = sm_context.second_counter;

        /* I2C bus fault: just probe the addresses, go back to INIT if found */
        if (sm_context.fault_source == SM_FAULT_I2C_BUS) {
            uart_printf("[SM] Retrying I2C bus...\n");
            gauge_init();
            if (I2C_TryAddress(I2C_0_INST, GAUGE_I2C_ADDR) &&
                I2C_TryAddress(I2C_0_INST, BQ25628E_I2C_ADDR)) {
                uart_printf("[SM] I2C devices found, returning to INIT\n");
                SM_Transition(SM_STATE_INIT);
            } else {
                uart_printf("[SM] I2C still unavailable\n");
            }
            return;
        }

        /* Gauge / charger faults: read live status and clear if clean */
        BQ27Z746_GetSafetyStatus(I2C_0_INST, &last_safety_status);
        last_charger_status = BQ25628E_GetFaultFlags();

        if (last_safety_status == 0 && last_charger_status == 0) {
            uart_printf("[SM] All safety flags clear\n");
            SM_Transition(SM_STATE_IDLE);
        } else {
            if (last_safety_status != 0) {
                uart_printf("[SM] Battery Fault Active: 0x%08X\n", (unsigned int)last_safety_status);
                SM_DecodeBatterySafetyStatus(last_safety_status);
            }
            if (last_charger_status != 0) {
                uart_printf("[SM] Charger Fault Active: 0x%02X\n", last_charger_status);
                SM_DecodeChargingSafetyStatus(last_charger_status);
            }
        }
    }
}

void SM_Run(void) {
    SM_Handle_RTC_Tick();
    switch (sm_context.current) {
        case SM_STATE_INIT:           SM_HandleState_INIT();           break;
        case SM_STATE_CHARGING:       SM_HandleState_CHARGING();       break;
        case SM_STATE_POWER_STM:      SM_HandleState_POWER_STM();      break;
        case SM_STATE_IDLE:           SM_HandleState_IDLE();           break;
        case SM_STATE_CRITICAL_FAULT: SM_HandleState_CRITICAL_FAULT(); break;
    }
}

bool SM_SafetyCheck(void) {
    if (sm_context.current == SM_STATE_CRITICAL_FAULT || 
        sm_context.current == SM_STATE_INIT) return false;

    uint32_t safety = 0;
    BQ27Z746_GetSafetyStatus(I2C_0_INST, &safety);
    if (safety != 0) {
        last_safety_status = safety; 
        if (SM_ProcessFault(safety, 0)) {
            sm_context.fault_source = SM_FAULT_GAUGE;
            SM_Transition(SM_STATE_CRITICAL_FAULT);
            return true; 
        }
        return false;  
    }
    return false;
}

bool SM_ChargingSafetyCheck(void) {
    uint8_t fault_flags = BQ25628E_GetFaultFlags();
    if (fault_flags != 0U) {
        last_charger_status = fault_flags;
        if (SM_ProcessFault(0, fault_flags)) {
            sm_context.fault_source = SM_FAULT_CHARGER;
            SM_Transition(SM_STATE_CRITICAL_FAULT);
            return true; 
        }
        return false; 
    }
    return false;
}

static bool SM_NeedsPeriodicSTMWake(SM_PowerContext_t pwr)
{
    if (pwr.is_critical_low && sm_context.critical_msg_sent) return false;
    
    // In PIR wake mode (wake_mode == 1), check the 24-hour lifeline timer
    if (sm_context.stm_wake_period.wake_mode == 1) {
        if ((sm_context.minute_counter - sm_context.last_lifeline_reset_minute) >= SM_LIFELINE_TIMEOUT_MINUTES) {
            sm_context.wake_reason = SM_WAKE_NORMAL; // Wake up with reason NORMAL
            sm_context.last_lifeline_reset_minute = sm_context.minute_counter; // Reset lifeline
            return true;
        }
        return false;
    }
    
    // Periodic wake mode (wake_mode == 0) legacy logic
    return (sm_context.minute_counter - sm_context.last_stm_periodic_minute) >= 
           sm_context.stm_wake_period.wake_interval_minutes;
}

static void SM_ResumeSystemContext(void) {
    SM_PowerContext_t pwr = SM_FetchPowerContext();
    if (pwr.is_charging) {
        uart_printf("[SM] Continuing to charge\n");
        SM_Transition(SM_STATE_CHARGING);
    } else if (pwr.is_critical_low) {
        uart_printf("[SM] Critical low battery detected, entering IDLE to conserve power\n");
        SM_Transition(SM_STATE_IDLE);
    } else {
        SM_Transition(SM_STATE_IDLE);
    }
}

/* ── Protocol helpers ───────────────────────────────────────────────────── */
static SM_SpiPacket_t* SM_InitDataPacket(SM_PayloadId_t pid) {
    SM_SpiPacket_t *pkt = (SM_SpiPacket_t *)stm32Spi.txBuf;
    memset(stm32Spi.txBuf, 0, stm32Spi.size);
    pkt->pkt.header.msg_type   = MSG_DATA;
    pkt->pkt.header.payload_id = pid;
    sm_context.has_pending_response = true;
    return pkt;
}

static void SM_PrepareSimpleMsg(SM_MsgType_t type)
{
    SM_SpiPacket_t *pkt = (SM_SpiPacket_t *)stm32Spi.txBuf;
    memset(stm32Spi.txBuf, 0, stm32Spi.size);

    pkt->pkt.header.msg_type   = type;
    pkt->pkt.header.payload_id = 0;
    pkt->pkt.header.length     = 0;
}

static void SM_SendOffer(void) { 
    SM_PrepareSimpleMsg(MSG_OFFER);
    SPI_Controller_Arm(&stm32Spi);
}

static void SM_PrepareAck(void) { 
    SM_PrepareSimpleMsg(MSG_ACK); 
    sm_context.has_pending_response = true;
}

static void SM_PrepareNack(void) { 
    SM_PrepareSimpleMsg(MSG_NACK);  
    sm_context.has_pending_response = true;
}

static void SM_PrepareTelemetryResponse(void)
{
    SM_PowerContext_t pwr = SM_FetchPowerContext();
    BQ27Z746_GetSafetyStatus(I2C_0_INST, &last_safety_status);

    uint8_t  chg_flags   = BQ25628E_ReadReg8(BQ25628E_REG_CHG_FLAG0);
    uint8_t  fault_flags = BQ25628E_ReadReg8(BQ25628E_REG_FAULT_FLAG0);

    uint16_t batt_status = BQ27Z746_Get_BatteryStatus();

    int btmp_dC = (int)(BQ25628E_Get_TBAT_C() * 10.0f);

    /* Report the state we came from, not POWER_STM which is always current here */
    const char* prev_state_str;
    switch (sm_context.previous) {
        case SM_STATE_INIT:           prev_state_str = "INIT";           break;
        case SM_STATE_CHARGING:       prev_state_str = "CHARGING";       break;
        case SM_STATE_POWER_STM:      prev_state_str = "POWER_STM";      break;
        case SM_STATE_IDLE:           prev_state_str = "IDLE";           break;
        case SM_STATE_CRITICAL_FAULT: prev_state_str = "CRITICAL_FAULT"; break;
        default:                      prev_state_str = "UNKNOWN";        break;
    }

    const char* wake_reason_str = "normal";
    if (sm_context.wake_reason == SM_WAKE_SETUP) wake_reason_str = "setup";
    else if (sm_context.wake_reason == SM_WAKE_PIR) wake_reason_str = "pir";

    int lux_val = 0;
    if (gLTR329.initialized) {
        uint16_t ch0, ch1;
        if (LTR329_ReadData(&ch0, &ch1)) {
            lux_val = (int)LTR329_CalculateLux(ch0, ch1);
        }
    }

    int accel_x = 0, accel_y = 0, accel_z = 0;
    if (gLIS3DH.initialized) {
        float x_mg, y_mg, z_mg;
        if (LIS3DH_ReadMg(&x_mg, &y_mg, &z_mg)) {
            accel_x = (int)x_mg;
            accel_y = (int)y_mg;
            accel_z = (int)z_mg;
        }
    }

    snprintf(json_buf, sizeof(json_buf),
        "{\"soc\":%d,\"soh\":%d,"
        "\"vbat\":%d,\"ibat\":%d,\"vchg\":%d,\"vsys\":%d,\"ichg\":%d,\"avgi\":%d,\"avgpwr\":%d,"
        "\"gtmp\":%d,\"ctmp\":%d,\"btmp\":%d,"
        "\"cycles\":%d,"
        "\"adapter\":%d,"
        "\"state\":\"%s\","
        "\"safety\":\"0x%08X\",\"battstat\":\"0x%04X\","
        "\"chgflags\":\"0x%02X\",\"faultflags\":\"0x%02X\","
        "\"chgstat\":\"%s\","
        "\"lowbattery\":%d,"
        "\"wake_interval\":%d,\"wake_mode\":%d,\"trigger\":\"%s\","
        "\"vreg\":%d,\"cfg_ichg\":%d,\"iindpm\":%d,"
        "\"vindpm\":%d,\"vsysmin\":%d,\"iprechg\":%d,\"iterm\":%d,"
        "\"lux\":%d,\"accel_x\":%d,\"accel_y\":%d,\"accel_z\":%d}",
        BQ27Z746_Get_SOC_pct(), BQ27Z746_Get_StateOfHealth_pct(),
        pwr.vbat_mv, BQ27Z746_Get_Current_mA(), BQ25628E_Get_VBUS_mV(),
        BQ25628E_Get_VSYS_mV(), BQ25628E_Get_IBUS_mA(), BQ27Z746_Get_AvgCurrent_mA(),
        BQ27Z746_Get_AvgPower_mW(), (int)BQ27Z746_Get_InternalTemp_C(),
        BQ25628E_Get_TDIE_C(), btmp_dC,
        BQ27Z746_Get_CycleCount(),
        pwr.adapter_present ? 1 : 0, prev_state_str,
        (unsigned int)last_safety_status, batt_status, chg_flags, fault_flags,
        SM_GetChargeString(pwr.chg_stat), pwr.is_critical_low ? 1 : 0,
        sm_context.stm_wake_period.wake_interval_minutes,
        sm_context.stm_wake_period.wake_mode, wake_reason_str,
        sm_context.sm_charger_config.vreg_mV,
        sm_context.sm_charger_config.ichg_mA,
        sm_context.sm_charger_config.iindpm_mA,
        sm_context.sm_charger_config.vindpm_mV,
        sm_context.sm_charger_config.vsysmin_mV,
        sm_context.sm_charger_config.iprechg_mA,
        sm_context.sm_charger_config.iterm_mA,
        lux_val, accel_x, accel_y, accel_z);

    SM_SpiPacket_t *pkt = SM_InitDataPacket(PID_TELEMETRY);
    size_t len = strlen(json_buf);
    if (len > sizeof(pkt->pkt.payload.telemetry.json))
        len = sizeof(pkt->pkt.payload.telemetry.json);
    pkt->pkt.header.length     = (uint16_t)len;

    /* The ~500-character JSON dump that used to be printed here is GONE, on
       purpose. It sat directly between receiving the STM32's request and
       staging the reply - so its entire transmission time was added to the
       round trip the STM32 is waiting on, and /mspm0 was timing out at 1000 ms
       and coming back as a 500 while the reply itself was fine.
       The STM32 logs the same JSON when it serves the page, so nothing is lost
       here that is not visible there. */
    memcpy(pkt->pkt.payload.telemetry.json, json_buf, len);

    if(pwr.is_critical_low) {
        sm_context.critical_msg_sent = true;
        uart_printf("[SM] Critical low battery detected, sending last packet till charge\n");
    }
}

static void SM_PrepareRTCResponse(void)
{
    RTC_GetTime(&sm_context.sm_rtc_config);
    SM_SpiPacket_t *pkt = SM_InitDataPacket(PID_RTC_GET);
    pkt->pkt.header.length     = sizeof(SM_RTCConfig_t);
    pkt->pkt.payload.rtc_data = sm_context.sm_rtc_config;
}

static void SM_HandleRequest(uint8_t pid)
{
    if (pid == PID_TELEMETRY) {
        SM_PrepareTelemetryResponse();
    } else if (pid == PID_RTC_GET) {
        SM_PrepareRTCResponse();
    } else if (pid == PID_STM_CFG) {
        SM_PrepareSTMConfigResponse();
    } else if (pid == PID_STM_CREDENTIALS) {
        SM_PrepareSTMCredentialsResponse();
    } else {
        SM_PrepareNack();
    }
}

static void SM_HandleConfig(uint8_t pid, const void *payload)
{
    if (pid == PID_CHARGER_CFG) {
        const SM_ChargerConfig_t *cfg = (const SM_ChargerConfig_t *)payload;
        BQ25628E_ApplyProfile(cfg);
        sm_context.sm_charger_config = *cfg;
        sm_context.charger_configured = true;
        SM_PrepareAck();
    } else if (pid == PID_RTC_SET) {
        const SM_RTCConfig_t *cfg = (const SM_RTCConfig_t *)payload;
        if (RTC_SetTime(cfg)) {
            sm_context.sm_rtc_config = *cfg;
            SM_PrepareAck();
        } else {
            SM_PrepareNack();
        }
    } else if (pid == PID_PERIOD_SET){
        const SM_PeriodConfig_t *cfg = (const SM_PeriodConfig_t *)payload;
        sm_context.stm_wake_period = *cfg;
        sm_context.wake_interval_configured = true;
        SM_PrepareAck();
    } else if (pid == PID_KEEP_ALIVE) {
        sm_context.last_io2_activity_s = sm_context.second_counter;
        SM_PrepareAck();
        uart_printf("[SM] Keep-alive received: resetting inactivity timer\n");
    } else if (pid == PID_POWER_CYCLE) {
        /* Do NOT cut power here. SM_PrepareAck() only stages the reply; it is
         * transmitted on the next transfer the STM32 initiates. Dropping the
         * rail now would kill it mid-request, and it would never learn that we
         * agreed. SM_HandleState_POWER_STM performs the cut once the reply has
         * actually gone out - see SM_DoPowerCycle(). */
        sm_context.last_io2_activity_s = sm_context.second_counter;
        sm_context.power_cycle_pending = true;
        SM_PrepareAck();
        uart_printf("[SM] Power-cycle requested by STM32 (deferred until ack sent)\n");
    } else if (pid == PID_STM_CFG) {
        const SM_STMConfig_t *cfg = (const SM_STMConfig_t *)payload;
        sm_context.stm_config = *cfg;
        sm_context.stm_config_received = true;
        uart_printf("[SM] STM config updated: conn=%d lte_baud=%d cam_res=%d\n",
            cfg->connectivity.mode,
            cfg->lte.baudrate_index,
            cfg->camera.resolution);
        SM_PrepareAck();
    } else if (pid == PID_STM_CREDENTIALS) {
        const SM_STMCredentials_t *creds = (const SM_STMCredentials_t *)payload;
        memcpy(&sm_context.stm_credentials, creds, sizeof(SM_STMCredentials_t));
        sm_context.stm_credentials_received = true;
        uart_printf("[SM] Credentials updated: ssid=%s device=%s\n",
            sm_context.stm_credentials.ap_ssid,
            sm_context.stm_credentials.device_name);
        SM_PrepareAck();
    } else {
        SM_PrepareNack();
    }
}

static void SM_PrepareSTMConfigResponse(void)
{
    SM_SpiPacket_t *pkt = SM_InitDataPacket(PID_STM_CFG);
    pkt->pkt.header.length     = sizeof(SM_STMConfig_t);
    pkt->pkt.payload.stm_config = sm_context.stm_config;

    uart_printf("[SM] STM config response sent (defaults: %s)\n",
        sm_context.stm_config_received ? "no" : "yes");
}

static void SM_PrepareSTMCredentialsResponse(void)
{
    SM_SpiPacket_t *pkt = SM_InitDataPacket(PID_STM_CREDENTIALS);
    pkt->pkt.header.length     = sizeof(SM_STMCredentials_t);
    pkt->pkt.payload.stm_credentials = sm_context.stm_credentials;

    uart_printf("[SM] Credentials response sent (defaults: %s)\n",
        sm_context.stm_credentials_received ? "no" : "yes");
}

static void SM_DispatchIncomingPacket(void)
{
    SM_MsgHeader_t *hdr = (SM_MsgHeader_t *)stm32Spi.rxBuf;

    switch (hdr->msg_type) {
        case MSG_REQUEST:
            SM_HandleRequest(hdr->payload_id);
            break;

        case MSG_CONFIG:
            SM_HandleConfig(hdr->payload_id, stm32Spi.rxBuf + sizeof(SM_MsgHeader_t));
            break;

        case MSG_SHUTDOWN:
            uart_printf("[SM] STM32 requested shutdown\n");
            SM_SetSTMPower(false);
            SM_ResumeSystemContext();
            return;

        default:
            /* IGNORE, DO NOT NACK. STAGING A REPLY TO A FRAME NOBODY ASKED
             * ABOUT IS WHAT MADE THIS FAULT SELF-SUSTAINING.
             *
             * Not every transfer carries a request. The STM32's response leg
             * transmits a dummy buffer while it reads our staged reply, an
             * abandoned transfer leaves an idle line, and an orphan clocks
             * whatever happens to be there. All of those land here.
             *
             * SM_PrepareNack() used to run for each of them, and it sets
             * has_pending_response - so we would sit holding a reply the STM32
             * had never asked for and would never collect. Its NEXT request
             * then took the staged-reply branch instead of the offer branch:
             * we shipped the stale NACK, and because that branch leaves
             * stm_data_sent false we never dispatched the new request at all.
             * The STM32 duly received an offer on its response leg and reported
             *
             *     Unexpected response pkt format, msg_type: 0x01
             *
             * which is exactly what the log shows on every request after the
             * first failure. One bad frame poisoned every exchange that
             * followed, and each timeout minted a fresh NACK to keep it going.
             *
             * A NACK is only meaningful as an answer to a request we understood
             * and refused. SM_HandleRequest() and SM_HandleConfig() already
             * NACK unknown payload ids, which is the case that deserves it.
             * Silence is the correct response to a frame that is not a request. */
            break;
    }
}

static void SM_DecodeBatterySafetyStatus(uint32_t status) {
    if (status & BQ27Z746_SAFETY_CUV)  uart_printf("[FAULT] CUV  : Cell Undervoltage\n");
    if (status & BQ27Z746_SAFETY_OVP)  uart_printf("[FAULT] COV  : Cell Overvoltage\n");
    if (status & BQ27Z746_SAFETY_OCC)  uart_printf("[FAULT] OCC  : Overcurrent During Charge\n");
    if (status & BQ27Z746_SAFETY_OCD)  uart_printf("[FAULT] OCD  : Overcurrent During Discharge\n");
    if (status & BQ27Z746_SAFETY_HOCD) uart_printf("[FAULT] HOCD : Overload During Discharge\n");
    if (status & BQ27Z746_SAFETY_HOCC) uart_printf("[FAULT] HOCC : Short-Circuit During Charge\n");
    if (status & BQ27Z746_SAFETY_SCD)  uart_printf("[FAULT] HSCD : Hardware Short-Circuit Discharge\n");
    if (status & BQ27Z746_SAFETY_OTC)  uart_printf("[FAULT] OTC  : Over-Temperature During Charge\n");
    if (status & BQ27Z746_SAFETY_OTD)  uart_printf("[FAULT] OTD  : Over-Temperature During Discharge\n");
    if (status & BQ27Z746_SAFETY_OTF)  uart_printf("[FAULT] OTF  : Over-Temperature FET\n");
    if (status & BQ27Z746_SAFETY_PTO)  uart_printf("[FAULT] PTO  : Precharge Timeout\n");
    if (status & BQ27Z746_SAFETY_CTO)  uart_printf("[FAULT] CTO  : Charge Timeout\n");
    if (status & BQ27Z746_SAFETY_UTC)  uart_printf("[FAULT] UTC  : Under-Temperature During Charge\n");
    if (status & BQ27Z746_SAFETY_UTD)  uart_printf("[FAULT] UTD  : Under-Temperature During Discharge\n");
    if (status & BQ27Z746_SAFETY_HCOV) uart_printf("[FAULT] HCOV : Hardware Cell Overvoltage\n");
    if (status & BQ27Z746_SAFETY_HCUV) uart_printf("[FAULT] HCUV : Hardware Cell Undervoltage\n");
}

static void SM_DecodeChargingSafetyStatus(uint8_t status) {
    if (status & BQ25628E_VBUS_FAULT_FLAG) uart_printf("  [!] VBUS: Over-Voltage or Sleep detected\n");
    if (status & BQ25628E_BAT_FAULT_FLAG)  uart_printf("  [!] BAT: Discharge OCP or VBAT OVP\n");
    if (status & BQ25628E_SYS_FAULT_FLAG) uart_printf("  [!] SYS: System Over-Voltage or Short Circuit\n");
    if (status & BQ25628E_TSHUT_FLAG) uart_printf(" [!] THERMAL: IC Thermal Shutdown triggered\n");
    if (status & BQ25628E_TS_FLAG) uart_printf("  [i] TS: Temperature status change detected\n");
}


void RTC_GetTime(SM_RTCConfig_t *out)
{
    DL_RTC_Calendar calendar;

    calendar = DL_RTC_getCalendarTime(RTC);

    out->second = calendar.seconds;
    out->minute = calendar.minutes;
    out->hour   = calendar.hours;
    out->day    = calendar.dayOfMonth;
    out->month  = calendar.month;
    out->year   = calendar.year; 
}

bool RTC_SetTime(const SM_RTCConfig_t *in)
{
    DL_RTC_Calendar calendar;

    if (in->second > 59 || in->minute > 59 || in->hour > 23 ||
        in->month < 1 || in->month > 12 ||
        in->day < 1 || in->day > 31 ||
        in->year < 2000 || in->year > 2099)
    {
        return false;
    }

    calendar.seconds    = in->second;
    calendar.minutes    = in->minute;
    calendar.hours      = in->hour;
    calendar.dayOfMonth = in->day;
    calendar.month      = in->month;
    calendar.year       = in->year;
    calendar.dayOfWeek  = 1;

    DL_RTC_initCalendar(RTC, calendar, DL_RTC_FORMAT_BINARY);
    return true;
}

static bool SM_ProcessFault(uint32_t gauge_safety, uint8_t charger_fault)
{
    bool is_critical = false;
    bool disable_charging = false;
    if (gauge_safety != 0) {
        SM_DecodeBatterySafetyStatus(gauge_safety);
        if (gauge_safety & (BQ27Z746_SAFETY_HOCC  |
                            BQ27Z746_SAFETY_SCD   |
                            BQ27Z746_SAFETY_HOCD)) {
            is_critical = true;
        }
        /* Recoverable faults : just disable charging, stay in current state */
        if (gauge_safety & (BQ27Z746_SAFETY_OVP   | BQ27Z746_SAFETY_HCOV |   // Overvoltage
                             BQ27Z746_SAFETY_OCC                           |   // Overcurrent charge
                             BQ27Z746_SAFETY_PTO  | BQ27Z746_SAFETY_CTO    |   // Timeouts
                             BQ27Z746_SAFETY_OTC  | BQ27Z746_SAFETY_UTC)) {    // Temperature charge
            disable_charging = true;
        }
    }

    /* ── Charger faults ─────────────────────────────────────── */
    if (charger_fault != 0) {
        SM_DecodeChargingSafetyStatus(charger_fault);

        /* Critical faults */
        if (charger_fault & (BQ25628E_SYS_FAULT_FLAG | BQ25628E_TSHUT_FLAG)) {
            is_critical = true;
        }
        /* VBUS_FAULT_FLAG : only real fault if adapter is actually present */
        if (charger_fault & BQ25628E_VBUS_FAULT_FLAG) {
            SM_PowerContext_t pwr = SM_FetchPowerContext();
            if (pwr.adapter_present) {
                disable_charging = true;
                uart_printf("[SM] VBUS fault while adapter present : charging disabled\n");
            } else {
                uart_printf("[SM] VBUS sleep (no adapter) : ignored\n");
            }
        }
        /* BAT_FAULT_FLAG : recoverable */
        if (charger_fault & BQ25628E_BAT_FAULT_FLAG) {
            disable_charging = true;
        }
    }

    /* Apply the recoverable action */
    if (disable_charging && !is_critical) {
        DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN);
        uart_printf("[SM] Recoverable fault : charging disabled (system continues running)\n");
    }

    return is_critical;
}