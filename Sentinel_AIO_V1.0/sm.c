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
#include "ics/IMX335/IMX335.h"   /* exposure/gain clamp limits for the AE seed */
#include "HAL/ticks.h"
#include <math.h>

/* ── PIR Configuration ───────────────────────────────────── */
#define SM_PIR_FILTER_STEP ZDP323B_FILTER_STEP_3
#define SM_PIR_FILTER_TYPE ZDP323B_FILTER_TYPE_C
#define SM_PIR_THRESHOLD   50

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
extern volatile uint32_t stm_io2_edge_us; /* ditto - IO2 edge arrival, us since the trigger */
extern volatile uint8_t  wake_trigger_src; /* 0 none, 1 PIR, 2 hall - latched in the ISR   */
extern volatile uint8_t  cam_sync_edges;   /* STM32 says "picture taken" - see main.c      */
extern volatile bool pir_monitor_active;
extern SPI_Controller_Handle stm32Spi;

/* ── Static variables ────────────────────────────────────── */
static uint32_t last_safety_status = 0;
static uint8_t last_charger_status = 0;
static char json_buf[600];

static bool ae_seed_sent;                     /* sent once per STM32 power-on  */
static bool ae_seed_awaiting_ack;             /* seed out, not yet confirmed   */

#define SM_AE_SEED_DEFER_MAX_MS   120U

static bool     ae_seed_deferred;             /* request held, ALS not ready   */
static uint32_t ae_seed_defer_edge_us;        /* the edge we are holding for   */
static bool ae_range_done;                    /* ALS gain settled for this wake */
static uint8_t ae_range_attempts;             /* bounded, see SM_AutoRangeAls  */

typedef struct {
    uint32_t sm_seen_us;       /* main loop reached the trigger check       */
    uint32_t rail_us;          /* STM32 rail high = the STM32's own t = 0   */
    uint8_t  trigger_src;      /* 1 PIR, 2 hall, 0 periodic (no edge)       */

    uint32_t als_wake_us;      /* LTR-329 taken out of standby              */
    uint32_t als_ranged_us;    /* auto-ranging settled                      */
    uint32_t build_start_us;   /* SM_BuildAeSeed() entered                  */
    uint32_t build_end_us;     /* seed computed                             */
    uint32_t edge_us;          /* FSBL's IO2 edge arrived (from the ISR)    */
    uint32_t arm_us;           /* SPI armed - FSBL's timeout stops here     */
    uint32_t done_us;          /* exchange complete, marker checked         */
    uint32_t fsbl_tick_ms;     /* STM32's own HAL_GetTick() at its toggle   */
    uint32_t pre_hal_ms;       /* derived: rail-up -> HAL_Init, invisible
                                * to the STM32 because its clock starts at
                                * the far end of it                         */
    uint16_t als_settle_ms;    /* how long BuildAeSeed waited on the ALS    */
    uint8_t  range_attempts;   /* gain changes spent                        */
    bool     valid;            /* a seed was actually built                 */
    bool     acked;            /* FSBL's marker came back                   */

    /* -- active illumination, recorded rather than printed. These events all
     *    land inside FSBL's seed window, where a blocking 9600-baud print is
     *    37-69 ms out of the deadline. See SM_AeWindowOpen(). -- */
    uint32_t led_on_us;        /* emitter lit (0 = not used this wake)      */
    uint32_t led_lux_milli;    /* lux the dark/light call was made on       */
    uint32_t cam_sync_us;      /* accepted CAM_SYNC edge                    */
    uint16_t led_on_ms;        /* how long the emitter actually burned      */
    uint8_t  cam_sync_rejected;/* early edges refused - SM_CamSyncPlausible */
    bool     led_used;
    bool     led_timeout;      /* burned to SM_LED_MAX_ON_MS, no CAM_SYNC   */
} SM_AeTiming_t;

static SM_AeTiming_t ae_time;

#define SM_LED_LUX_THRESHOLD        10.0f
#define SM_LED_CURRENT_MA           1000U
#define SM_LED_SEED_GAIN_MDB        40000U
#define SM_LED_AWB_COLOR_TEMP       4015U
#define SM_LED_MAX_ON_MS            250U

#define SM_CAM_SYNC_LED_BLANKING_MS  30U
#define SM_CAM_SYNC_FLOOR_MS        400U

typedef struct {
    bool     decided;        /* this wake's dark/light call has been made   */
    bool     on;             /* emitter is lit right now                    */
    bool     use_led_seed;   /* seed the fixed pair instead of the model    */
    uint32_t on_at_ms;       /* Ticks_ms() when it was lit, for the timeout */
    uint16_t ch0, ch1;
    uint8_t  als_gain;
    float    lux;
    bool     snapshot_valid;
} SM_Illum_t;

static SM_Illum_t illum;

typedef struct {
    uint16_t    ch0, ch1;
    LTR329_Gain gain;        /* gain the counts were actually taken at */
    bool        valid;
} SM_AlsSample_t;

static SM_AlsSample_t als_last;

/** @brief Latch a conversion. Called wherever one is successfully read. */
static void SM_AlsLatch(uint16_t ch0, uint16_t ch1)
{
    als_last.ch0   = ch0;
    als_last.ch1   = ch1;
    als_last.gain  = gLTR329.gain;
    als_last.valid = true;
}

/** @brief True when the latched sample is still scale-correct to use. */
static bool SM_AlsSampleUsable(void)
{
    return (als_last.valid && (als_last.gain == gLTR329.gain));
}

static bool SM_AeWindowOpen(void)
{
    return ((!ae_seed_sent) || ae_seed_awaiting_ack);
}

/* Something was suppressed by the window and still owes the console a line. */
static bool ae_notes_pending;
static const char *ae_fail_reason;
static uint8_t     ae_fail_status;
static uint16_t    ae_fail_waited_ms;

static void SM_IlluminationOff(void);
static void SM_IlluminationDecide(void);
static void SM_IlluminationService(void);

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
static bool SM_BuildAeSeed(SM_AeSeedPayload_t *seed);
static void SM_SendAeSeed(void);
static void SM_AutoRangeAls(void);
static bool SM_CheckExternalWakeTriggers(void);

static void SM_Heartbeat(void);

static void SM_HandleState_INIT(void);
static void SM_HandleState_CHARGING(void);
static void SM_HandleState_POWER_STM(void);
static void SM_HandleState_IDLE(void);
static void SM_HandleState_CRITICAL_FAULT(void);


/* ── Heartbeat ───────────────────────────────────────────── */
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
        if (Ticks_StartIfIdle()) {
            wake_trigger_src = 0U;    /* periodic - no interrupt to attribute */
        }
        ae_time.rail_us     = Ticks_us();
        ae_time.trigger_src = wake_trigger_src;
    } else {
        SM_IlluminationOff();
        Ticks_Stop();
        wake_trigger_src = 0U;
        LTR329_SetMode(false);
        DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN | DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
        DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_STM_MCU_IO1_PIN);
        PWR_ExitActiveProfile();
    }
}

#define SM_MS_TO_CYCLES(ms)   ((uint32_t)(ms) * 32000U)

#define SM_ALS_POLL_STEP_MS   2U
#define SM_ALS_POLL_CAP_MS    30U

#define SM_AWB_CT_INCANDESCENT   2810U   /* JudgeII-A     */
#define SM_AWB_CT_FLUORESCENT    4015U   /* JudgeII-TL84  */
#define SM_AWB_CT_DAYLIGHT       6650U   /* JudgeII-DAY   */

#define SM_IR_RATIO_INCANDESCENT   0.5649f  /* ratio >= 0.5649 -> 2810K */
#define SM_IR_RATIO_DAYLIGHT       0.4904f  /* ratio <= 0.4904 -> 6650K */
#define SM_LUX_NEAR_DARK           20.0f


#define SM_ALS_COUNT_TARGET   8000U
#define SM_ALS_COUNT_OK_LO    1000U
#define SM_ALS_COUNT_OK_HI   40000U
#define SM_ALS_COUNT_SAT     60000U
#define SM_ALS_RANGE_MAX_ATTEMPTS  2U

static const LTR329_Gain sm_als_gains[] = {
    LTR329_GAIN_1X, LTR329_GAIN_2X,  LTR329_GAIN_4X,
    LTR329_GAIN_8X, LTR329_GAIN_48X, LTR329_GAIN_96X
};
#define SM_ALS_GAIN_COUNT  (sizeof(sm_als_gains) / sizeof(sm_als_gains[0]))

static LTR329_Gain SM_AlsStatusGain(uint8_t status)
{
    switch ((status >> 4) & 0x07U) {
        case 0x00: return LTR329_GAIN_1X;
        case 0x01: return LTR329_GAIN_2X;
        case 0x02: return LTR329_GAIN_4X;
        case 0x03: return LTR329_GAIN_8X;
        case 0x06: return LTR329_GAIN_48X;
        case 0x07: return LTR329_GAIN_96X;
        default:   return LTR329_GAIN_1X;   /* 0x04/0x05 are marked invalid */
    }
}

static LTR329_Gain SM_PickAlsGain(LTR329_Gain current, uint16_t ch0)
{
    /* Clipped — the count carries no scale information, so just back off one
     * step and look again. */
    if (ch0 >= SM_ALS_COUNT_SAT) {
        for (unsigned i = SM_ALS_GAIN_COUNT - 1U; i > 0U; i--) {
            if (sm_als_gains[i] == current) return sm_als_gains[i - 1U];
        }
        return LTR329_GAIN_1X;
    }

    /* Nothing at all: jump straight to maximum sensitivity. */
    if (ch0 == 0U) {
        return LTR329_GAIN_96X;
    }

    uint32_t desired = ((uint32_t)current * SM_ALS_COUNT_TARGET) / (uint32_t)ch0;

    /* Snap *down* to the largest available gain not exceeding the ideal, so we
     * always land under the target. Overshooting risks saturation, which costs
     * another conversion to detect and undo. */
    LTR329_Gain best = LTR329_GAIN_1X;
    for (unsigned i = 0U; i < SM_ALS_GAIN_COUNT; i++) {
        if ((uint32_t)sm_als_gains[i] <= desired) best = sm_als_gains[i];
    }
    return best;
}

static void SM_AutoRangeAls(void)
{
    uint8_t  status = 0;
    uint16_t ch0 = 0, ch1 = 0;

    if (ae_range_done || !gLTR329.initialized) {
        return;
    }

    if (!LTR329_GetStatus(&status)) {
        return;                       /* bus busy or absent; retry next pass */
    }

    if (((status & LTR329_STATUS_INVALID) != 0U) &&
        (SM_AlsStatusGain(status) == gLTR329.gain)) {

        LTR329_Gain lower = gLTR329.gain;
        for (unsigned i = SM_ALS_GAIN_COUNT - 1U; i > 0U; i--) {
            if (sm_als_gains[i] == gLTR329.gain) { lower = sm_als_gains[i - 1U]; break; }
        }

        if ((lower != gLTR329.gain) && LTR329_SetGain(lower)) {
            return;                   /* next pass reads at the new gain */
        }

        ae_range_done          = true;
        ae_time.als_ranged_us  = Ticks_us();
        ae_time.range_attempts = ae_range_attempts;
        return;
    }

    /* Only act on a completed conversion actually taken at the gain currently
     * programmed - otherwise we would range off a measurement from before the
     * last change. */
    if (((status & LTR329_STATUS_NEW_DATA) == 0U) ||
        (SM_AlsStatusGain(status) != gLTR329.gain)) {
        return;
    }

    if (!LTR329_ReadData(&ch0, &ch1)) {
        return;
    }

    SM_AlsLatch(ch0, ch1);

    if (((ch0 >= SM_ALS_COUNT_OK_LO) && (ch0 <= SM_ALS_COUNT_OK_HI)) ||
        (ae_range_attempts >= SM_ALS_RANGE_MAX_ATTEMPTS)) {
        ae_range_done = true;
        ae_time.als_ranged_us  = Ticks_us();
        ae_time.range_attempts = ae_range_attempts;
        return;
    }

    LTR329_Gain next = SM_PickAlsGain(gLTR329.gain, ch0);

    if (next == gLTR329.gain) {
        ae_range_done = true;
        ae_time.als_ranged_us  = Ticks_us();
        ae_time.range_attempts = ae_range_attempts;
        return;
    }

    if (LTR329_SetGain(next)) {
        ae_range_attempts++;
    } else {
        ae_range_done = true;         /* cannot talk to it; stop trying */
        ae_time.als_ranged_us  = Ticks_us();
        ae_time.range_attempts = ae_range_attempts;
        uart_printf("[AE] ALS gain write failed, keeping %uX\n",
                    (unsigned)gLTR329.gain);
    }
}

static uint32_t SM_PickAwbColorTemp(uint16_t ch0, uint16_t ch1, float lux)
{
    uint32_t denom = (uint32_t)ch0 + (uint32_t)ch1;
    if ((denom == 0U) || (lux < SM_LUX_NEAR_DARK)) {
        return SM_AWB_CT_INCANDESCENT;
    }

    float ratio = (float)ch1 / (float)denom;

    if (ratio >= SM_IR_RATIO_INCANDESCENT) return SM_AWB_CT_INCANDESCENT;
    if (ratio <= SM_IR_RATIO_DAYLIGHT)     return SM_AWB_CT_DAYLIGHT;
    return SM_AWB_CT_FLUORESCENT;
}

static bool SM_CamSyncPlausible(void)
{
    uint32_t now_ms  = Ticks_ms();
    uint32_t rail_ms = ae_time.rail_us / 1000U;
    uint32_t since_rail_ms = (now_ms >= rail_ms) ? (now_ms - rail_ms) : now_ms;

    if (illum.on && ((now_ms - illum.on_at_ms) < SM_CAM_SYNC_LED_BLANKING_MS)) {
        return false;
    }

    if (ae_seed_sent && !ae_seed_awaiting_ack) {
        return true;
    }

    return (since_rail_ms >= SM_CAM_SYNC_FLOOR_MS);
}

static void SM_IlluminationOff(void)
{
    bool was_on = illum.on;
    uint32_t on_ms = was_on ? (Ticks_ms() - illum.on_at_ms) : 0U;
    LED_off();
    illum.on = false;

    if (was_on) {
        ae_time.led_on_ms = (uint16_t)on_ms;

        if (SM_AeWindowOpen()) {
            ae_notes_pending = true;
        } else {
            uart_printf("[LED] off after %lums\n", (unsigned long)on_ms);
        }
    }
}


static void SM_IlluminationDecide(void)
{
    uint16_t ch0 = 0, ch1 = 0;
    uint8_t  status = 0;

    if (illum.decided || !ae_range_done || !gLTR329.initialized) {
        return;
    }

    if (SM_AlsSampleUsable()) {
        ch0 = als_last.ch0;
        ch1 = als_last.ch1;
    }
    else {
        if (!LTR329_GetStatus(&status)) {
            return;
        }
        if (((status & LTR329_STATUS_NEW_DATA) == 0U) ||
            ((status & LTR329_STATUS_INVALID)  != 0U) ||
            (SM_AlsStatusGain(status) != gLTR329.gain)) {
            return;                      /* not ready - try again next pass */
        }
        if (!LTR329_ReadData(&ch0, &ch1)) {
            return;
        }
        SM_AlsLatch(ch0, ch1);
    }

    illum.ch0            = ch0;
    illum.ch1            = ch1;
    illum.als_gain       = (uint8_t)gLTR329.gain;
    illum.lux            = LTR329_CalculateLux(ch0, ch1);
    illum.snapshot_valid = true;
    illum.decided        = true;

    if (illum.lux < SM_LED_LUX_THRESHOLD) {
        illum.use_led_seed = true;

        LED_set_voltage(7000U);
        LED_set_current(SM_LED_CURRENT_MA);
        enable_led_boost();

        illum.on       = true;
        illum.on_at_ms = Ticks_ms();

        uint32_t lux_milli = (uint32_t)(illum.lux * 1000.0f);

        ae_time.led_on_us     = Ticks_us();
        ae_time.led_lux_milli = lux_milli;
        ae_time.led_used      = true;
        if (SM_AeWindowOpen()) {
            ae_notes_pending = true;
        } else {
            uart_printf("[LED] on %umA at %lums (lux %lu.%03lu)\n",
                        (unsigned)SM_LED_CURRENT_MA,
                        (unsigned long)illum.on_at_ms,
                        (unsigned long)(lux_milli / 1000U),
                        (unsigned long)(lux_milli % 1000U));
        }
    }
}

static void SM_IlluminationService(void)
{
    uint8_t edges;

    __disable_irq();
    edges = cam_sync_edges;
    cam_sync_edges = 0U;
    __enable_irq();

    if ((edges > 0U) && !SM_CamSyncPlausible()) {
        if (ae_time.cam_sync_rejected < 255U) {
            ae_time.cam_sync_rejected++;
        }
        ae_notes_pending = true;
        edges = 0U;
    }

    if (edges > 0U) {
        uint32_t now_ms = Ticks_ms();
        uint32_t rail_ms = ae_time.rail_us / 1000U;
        uint32_t elapsed_ms = (now_ms >= rail_ms) ? (now_ms - rail_ms) : now_ms;

        ae_time.cam_sync_us = Ticks_us();
        if (SM_AeWindowOpen()) {
            ae_notes_pending = true;
        } else {
            uart_printf("[CAM_SYNC] picture taken at %lums (%lums from rail-up, LED %s)\n",
                        (unsigned long)now_ms,
                        (unsigned long)elapsed_ms,
                        illum.on ? "ON" : "OFF");
        }

        if (illum.on) {
            SM_IlluminationOff();            /* picture taken - turn off emitter */
        }
        return;
    }

    if (illum.on && ((Ticks_ms() - illum.on_at_ms) >= SM_LED_MAX_ON_MS)) {
        ae_time.led_timeout = true;

        if (SM_AeWindowOpen()) {
            ae_notes_pending = true;
        } else {
            uart_printf("[LED] TIMEOUT - no CAM_SYNC in %ums\n",
                        (unsigned)SM_LED_MAX_ON_MS);
        }
        SM_IlluminationOff();
    }
}

static bool SM_BuildAeSeed(SM_AeSeedPayload_t *seed)
{
    uint16_t ch0 = 0, ch1 = 0;
    uint8_t  status = 0;
    uint16_t waited_ms = 0U;

    ae_time.build_start_us = Ticks_us();

    ae_fail_reason    = NULL;
    ae_fail_status    = 0U;
    ae_fail_waited_ms = 0U;

    memset(seed, 0, sizeof(*seed));

    if (illum.use_led_seed && illum.snapshot_valid) {
        seed->exposure_us    = (uint32_t)IMX335_EXPOSURE_MAX;
        seed->gain_mdB       = SM_LED_SEED_GAIN_MDB;
        seed->awb_color_temp = SM_LED_AWB_COLOR_TEMP;
        seed->lux_milli      = (uint32_t)(illum.lux * 1000.0f);
        seed->als_ch0        = illum.ch0;
        seed->als_ch1        = illum.ch1;
        seed->als_settle_ms  = 0U;
        seed->valid          = 1U;
        seed->als_gain       = illum.als_gain;
        seed->pir_elapsed_ms = Ticks_us() / 1000U;

        ae_time.build_end_us = Ticks_us();
        ae_time.valid        = true;
        return true;
    }

    if (!gLTR329.initialized) {
        ae_fail_reason = "ALS not initialised";
        return false;
    }

    if (SM_AlsSampleUsable()) {
        ch0 = als_last.ch0;
        ch1 = als_last.ch1;
    }
    else {
        for (;;) {
            if (!LTR329_GetStatus(&status)) {
                ae_fail_reason = "ALS status read failed";
                return false;
            }
            if (((status & LTR329_STATUS_NEW_DATA) != 0U) &&
                ((status & LTR329_STATUS_INVALID)  == 0U) &&
                (SM_AlsStatusGain(status) == gLTR329.gain)) {
                break;
            }

            if (waited_ms >= SM_ALS_POLL_CAP_MS) {
                ae_fail_reason    = "ALS not ready";
                ae_fail_status    = status;
                ae_fail_waited_ms = waited_ms;
                return false;
            }

            delay_cycles(SM_MS_TO_CYCLES(SM_ALS_POLL_STEP_MS));
            waited_ms = (uint16_t)(waited_ms + SM_ALS_POLL_STEP_MS);
        }

        if (!LTR329_ReadData(&ch0, &ch1)) {
            ae_fail_reason = "ALS read failed";
            return false;
        }

        SM_AlsLatch(ch0, ch1);
    }

    if ((ch0 == 0U) && (ch1 == 0U)) {
        ae_fail_reason = "ALS no signal on either channel";
        return false;
    }

    float lux = LTR329_CalculateLux(ch0, ch1);

    double model_in  = log1p((double)lux);
    double exposure  = score_exposure_sep(&model_in);
    double gain      = score_gain_sep(&model_in);

    if (exposure < 0.0) exposure = 0.0;
    if (gain     < 0.0) gain     = 0.0;

    uint32_t exposure_us = (uint32_t)exposure;
    uint32_t gain_mdB    = (uint32_t)gain;

    if (exposure_us > (uint32_t)IMX335_EXPOSURE_MAX) exposure_us = (uint32_t)IMX335_EXPOSURE_MAX;
    if (gain_mdB    > (uint32_t)IMX335_GAIN_MAX)     gain_mdB    = (uint32_t)IMX335_GAIN_MAX;

    seed->exposure_us    = exposure_us;
    seed->gain_mdB       = gain_mdB;
    seed->awb_color_temp = SM_PickAwbColorTemp(ch0, ch1, lux);
    seed->lux_milli      = (uint32_t)(lux * 1000.0f);
    seed->als_ch0        = ch0;
    seed->als_ch1        = ch1;
    seed->als_settle_ms  = waited_ms;
    seed->valid          = 1U;
    seed->als_gain       = (uint8_t)gLTR329.gain;
    seed->pir_elapsed_ms = Ticks_us() / 1000U;

    ae_time.build_end_us   = Ticks_us();
    ae_time.als_settle_ms  = waited_ms;
    ae_time.valid          = true;

    return true;
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
    /* Do not block on 9600-baud uart_printf (~28 ms) when transitioning to POWER_STM,
     * so the time-critical wake path is unblocked. */
    if (new_state != SM_STATE_POWER_STM) {
        uart_printf("[SM] %s -> %s\n", old_name, SM_GetStateString());
    }
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
    if (hall_wakeup_flag || pir_monitor_active) {
        ae_time.sm_seen_us = Ticks_us();
    }

    if (hall_wakeup_flag) {
        hall_wakeup_flag = false;
        sm_context.wake_reason = SM_WAKE_SETUP;
        RTC_EnablePrescaler();
        PWR_EnterMeasureProfile();
        /* Assert STM32 power rail and wake LTR329 immediately at t = 0 */
        SM_SetSTMPower(true);
        LTR329_SetMode(true);
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
            /* Assert STM32 power rail and wake LTR329 immediately at t = 0 */
            SM_SetSTMPower(true);
            LTR329_SetMode(true);
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

#define SM_POWER_CYCLE_OFF_MS     500U

/* Breathing room after the acknowledgement lands, before the rail drops. */
#define SM_POWER_CYCLE_SETTLE_MS   50U

static void SM_DoPowerCycle(void) {
    sm_context.power_cycle_pending = false;
    sm_context.power_cycle_armed   = false;

    uart_printf("[SM] Power-cycling STM32 (reason kept: %s)\n",
        (sm_context.wake_reason == SM_WAKE_SETUP) ? "SETUP" : "NORMAL");

    delay_cycles(SM_MS_CYCLES(SM_POWER_CYCLE_SETTLE_MS));

    SM_SetSTMPower(false);
    delay_cycles(SM_MS_CYCLES(SM_POWER_CYCLE_OFF_MS));
    sm_context.entry_done = false;
}

static void SM_HandleState_POWER_STM(void) {

    if (!sm_context.entry_done) {
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
        if (gLTR329.initialized) {
            (void)LTR329_SetTiming(LTR329_INT_50MS, 50U);
        }

        sm_context.stm_power_on_s = sm_context.second_counter;
        sm_context.last_io2_activity_s = sm_context.second_counter;
        sm_context.entry_done = true;
        sm_context.stm_data_sent = false;
        ae_seed_sent = false;   /* FSBL gets the camera seed on the first IO2 */
        ae_seed_awaiting_ack = false;
        ae_seed_deferred = false;       /* a hold belongs to one power-on only */
        ae_seed_defer_edge_us = 0U;
        ae_range_done = false;  /* re-range the ALS for this wake's lighting  */
        ae_range_attempts = 0U;
        als_last.valid = false;
        ae_fail_reason = NULL;

        ae_notes_pending          = false;
        ae_time.led_on_us         = 0U;
        ae_time.led_lux_milli     = 0U;
        ae_time.cam_sync_us       = 0U;
        ae_time.led_on_ms         = 0U;
        ae_time.cam_sync_rejected = 0U;
        ae_time.led_used          = false;
        ae_time.led_timeout       = false;

        ae_time.als_ranged_us  = 0U;
        ae_time.build_start_us = 0U;
        ae_time.build_end_us   = 0U;
        ae_time.edge_us        = 0U;
        ae_time.arm_us         = 0U;
        ae_time.done_us        = 0U;
        ae_time.fsbl_tick_ms   = 0U;
        ae_time.pre_hal_ms     = 0U;
        ae_time.als_settle_ms  = 0U;
        ae_time.range_attempts = 0U;
        ae_time.valid          = false;
        ae_time.acked          = false;

        ae_time.als_wake_us = Ticks_us();
        stm_io2_edge_us = 0U;

        memset(&illum, 0, sizeof(illum));
        cam_sync_edges = 0U;
        DL_GPIO_clearInterruptStatus(EXTERNAL_INTERRUPT_CAM_SYNC_PORT,
                                     EXTERNAL_INTERRUPT_CAM_SYNC_PIN);
        DL_GPIO_enableInterrupt(EXTERNAL_INTERRUPT_CAM_SYNC_PORT,
                                EXTERNAL_INTERRUPT_CAM_SYNC_PIN);
        sm_context.has_pending_response = false;

        DL_GPIO_disableInterrupt(EXTERNAL_INTERRUPT_CHARGER_INT_PORT, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        DL_GPIO_enableInterrupt(EXTERNAL_INTERRUPT_STM_MCU_IO2_PORT, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
        stm_io2_edges = 0U;
    }

    SM_AutoRangeAls();
    SM_IlluminationDecide();
    SM_IlluminationService();

    if (ae_seed_deferred)
    {
        uint32_t held_us = Ticks_us() - ae_seed_defer_edge_us;
        bool     expired = (held_us >= (SM_AE_SEED_DEFER_MAX_MS * 1000U));

        if (ae_range_done || expired)
        {
            ae_seed_deferred     = false;
            SM_SendAeSeed();          /* builds from the latch, or fails honestly */
            ae_seed_sent         = true;
            ae_seed_awaiting_ack = true;

            uart_printf("[AE] held %lums -> %s\n",
                        (unsigned long)(held_us / 1000U),
                        expired ? "TIMEOUT, sending anyway" : "ranged, sent");
        }
    }

    if (ae_seed_awaiting_ack && stm32Spi.rxDone) {
        ae_seed_awaiting_ack = false;
        stm32Spi.rxDone = false;

        ae_time.done_us = Ticks_us();

        if ((stm32Spi.rxBuf[0] == SM_AE_SEED_FSBL_MAGIC_0) &&
            (stm32Spi.rxBuf[1] == SM_AE_SEED_FSBL_MAGIC_1) &&
            (stm32Spi.rxBuf[2] == SM_AE_SEED_FSBL_MAGIC_2) &&
            (stm32Spi.rxBuf[3] == SM_AE_SEED_FSBL_MAGIC_3)) {
            ae_time.acked = true;

            ae_time.fsbl_tick_ms = ((uint32_t)stm32Spi.rxBuf[4])        |
                                   ((uint32_t)stm32Spi.rxBuf[5] << 8)   |
                                   ((uint32_t)stm32Spi.rxBuf[6] << 16)  |
                                   ((uint32_t)stm32Spi.rxBuf[7] << 24);

            {
                uint32_t rail_to_edge_ms =
                    (ae_time.edge_us - ae_time.rail_us) / 1000U;

                ae_time.pre_hal_ms = (rail_to_edge_ms >= ae_time.fsbl_tick_ms)
                                   ? (rail_to_edge_ms - ae_time.fsbl_tick_ms)
                                   : 0U;
            }

            /* One short line on the hot path. The table is `sm timing`. */
            uart_printf("[AE] ok %lums\n",
                        (unsigned long)(ae_time.done_us / 1000U));
        } else {
            uart_printf("[AE] seed NOT taken by FSBL (got %02X %02X %02X %02X) "
                        "- recovering, treating as normal traffic\n",
                        stm32Spi.rxBuf[0], stm32Spi.rxBuf[1],
                        stm32Spi.rxBuf[2], stm32Spi.rxBuf[3]);
            SM_DispatchIncomingPacket();
            if (sm_context.current != SM_STATE_POWER_STM) {
                return;
            }
        }
    }

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

        uart_printf(" t=%lu staged=%u\n",
            (unsigned long)sm_context.second_counter,
            (unsigned int)sm_context.has_pending_response);

        if (sm_context.current != SM_STATE_POWER_STM) {
            return;
        }
     }

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

        const char *io2_action;

        if (!ae_seed_sent && !ae_range_done) {
            ae_seed_deferred      = true;
            ae_seed_defer_edge_us = stm_io2_edge_us;
            io2_action            = "AE seed HELD (ALS ranging)";
        } else if (!ae_seed_sent) {
            SM_SendAeSeed();
            ae_seed_sent = true;
            ae_seed_awaiting_ack = true;
            io2_action = "AE seed -> FSBL";
        } else if (sm_context.has_pending_response) {
            SPI_Controller_Arm(&stm32Spi);
            sm_context.has_pending_response = false;
            io2_action = "staged reply";

            if (sm_context.power_cycle_pending) {
                sm_context.power_cycle_armed = true;
            }
        } else {
            SM_SendOffer();
            sm_context.stm_data_sent = true;
            io2_action = "offer";
        }

        /* Now that the transfer is on the wire, the UART can have the CPU. */
        uart_printf("[SM] t=%lu IO2: %s\n",
            (unsigned long)sm_context.second_counter, io2_action);
    }

    if (ae_notes_pending && !SM_AeWindowOpen()) {
        ae_notes_pending = false;

        if (ae_time.led_used) {
            uart_printf("[LED] lit at %lums (lux %lu.%03lu)\n",
                (unsigned long)(ae_time.led_on_us / 1000U),
                (unsigned long)(ae_time.led_lux_milli / 1000U),
                (unsigned long)(ae_time.led_lux_milli % 1000U));
        }
        if (ae_time.cam_sync_rejected > 0U) {
            uart_printf("[CAM_SYNC] %u early edge(s) refused\n",
                (unsigned)ae_time.cam_sync_rejected);
        }
    }

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
    if (SM_CheckExternalWakeTriggers()) return;

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


static void SM_SendAeSeed(void)
{
    SM_SpiPacket_t *pkt = (SM_SpiPacket_t *)stm32Spi.txBuf;
    SM_AeSeedPayload_t seed;
    bool ok = SM_BuildAeSeed(&seed);

    memset(stm32Spi.txBuf, 0, stm32Spi.size);

    pkt->pkt.header.msg_type   = MSG_DATA;
    pkt->pkt.header.payload_id = PID_AE_SEED;
    pkt->pkt.header.length     = sizeof(SM_AeSeedPayload_t);
    pkt->pkt.payload.ae_seed   = seed;   /* zeroed with valid == 0 on failure */

    SPI_Controller_Arm(&stm32Spi);

    ae_time.arm_us = Ticks_us();
    ae_time.edge_us = stm_io2_edge_us;

    if (!ok) {
        uart_printf("[AE] seed INVALID: %s (st 0x%02X %ums) - blind start\n",
                    (ae_fail_reason != NULL) ? ae_fail_reason : "unknown",
                    (unsigned)ae_fail_status,
                    (unsigned)ae_fail_waited_ms);
    }
}

void SM_PrintAeTiming(void)
{
    const char *src;

    if (ae_time.rail_us == 0U && ae_time.arm_us == 0U) {
        uart_printf("[AE] no wake recorded yet\n");
        return;
    }

    src = (ae_time.trigger_src == 1U) ? "PIR" :
          (ae_time.trigger_src == 2U) ? "hall/setup" : "periodic (no edge)";

    uart_printf("=== wake timeline (us since %s) ===\n", src);
    uart_printf("  edge -> SM     %8lu\n", (unsigned long)ae_time.sm_seen_us);
    uart_printf("  STM32 rail up  %8lu\n", (unsigned long)ae_time.rail_us);
    uart_printf("  ALS woken      %8lu\n", (unsigned long)ae_time.als_wake_us);
    uart_printf("  ALS ranged     %8lu  (%u gain change%s)\n",
                (unsigned long)ae_time.als_ranged_us,
                (unsigned)ae_time.range_attempts,
                (ae_time.range_attempts == 1U) ? "" : "s");
    uart_printf("  seed build     %8lu -> %lu  (ALS wait %u ms)\n",
                (unsigned long)ae_time.build_start_us,
                (unsigned long)ae_time.build_end_us,
                (unsigned)ae_time.als_settle_ms);
    uart_printf("  IO2 edge       %8lu\n", (unsigned long)ae_time.edge_us);
    uart_printf("  SPI armed      %8lu\n", (unsigned long)ae_time.arm_us);
    uart_printf("  exchange done  %8lu\n", (unsigned long)ae_time.done_us);

    /* This MCU's share of the budget, stated as one number. */
    uart_printf("  ---- PIR -> rail  %lu ms  (of a 200 ms budget)\n",
                (unsigned long)(ae_time.rail_us / 1000U));

    if (ae_time.arm_us >= ae_time.edge_us) {
        uint32_t margin_us = ae_time.arm_us - ae_time.edge_us;
        /* KEEP IN STEP WITH AE_SEED_TIMEOUT_MS in the STM32's ae_seed.h.
         * Raised 60000 -> 150000 on 2026-08-25 with that constant. */
        uart_printf("  edge -> arm    %8lu us   (budget 150000, %s)\n",
                    (unsigned long)margin_us,
                    (margin_us > 150000UL) ? "OVER - seed was lost" :
                    (margin_us >  90000UL) ? "TIGHT"               : "ok");
    }

    if (ae_time.acked) {
        uart_printf("  STM32 tick at edge %lu ms, rail -> edge %lu ms\n",
                    (unsigned long)ae_time.fsbl_tick_ms,
                    (unsigned long)((ae_time.edge_us - ae_time.rail_us) / 1000U));
        uart_printf("  ---- pre-HAL (BootROM+startup) %lu ms\n",
                    (unsigned long)ae_time.pre_hal_ms);
    } else {
        uart_printf("  seed NOT acknowledged by FSBL%s\n",
                    ae_time.valid ? "" : " (and the seed was invalid)");
    }

    if (ae_time.led_used) {
        uart_printf("  ---- emitter lit at %lu us, lux %lu.%03lu, burned %u ms%s\n",
                    (unsigned long)ae_time.led_on_us,
                    (unsigned long)(ae_time.led_lux_milli / 1000U),
                    (unsigned long)(ae_time.led_lux_milli % 1000U),
                    (unsigned)ae_time.led_on_ms,
                    ae_time.led_timeout ? " (TIMED OUT - no CAM_SYNC)" : "");
    }
    if (ae_time.cam_sync_us != 0U) {
        uart_printf("  ---- CAM_SYNC accepted at %lu us\n",
                    (unsigned long)ae_time.cam_sync_us);
    }
    if (ae_time.cam_sync_rejected > 0U) {
        uart_printf("  ---- CAM_SYNC refused %u early edge(s)\n",
                    (unsigned)ae_time.cam_sync_rejected);
    }
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