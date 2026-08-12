#ifndef FUNC_HELPERS_H
#define FUNC_HELPERS_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>
#include "emulation_type_b/eeprom_emulation_type_b.h"
#include "sm.h"


#define SM_SLEEP_WAKEUP_MINUTES   4

/* ─────────────────────────────────────────────────────────────────────────────
 * Power Profiles
 *
 *  MINIMUM   – Only RTC alive. Used during IDLE and CHARGING sleep windows.
 *              Wake sources: RTC minute tick, Hall GPIO interrupt.
 *
 *  MEASURE   – RTC + I2C_0. Used for periodic safety / charging reads inside
 *              IDLE and CHARGING states before returning to sleep.
 *
 *  ACTIVE    – Everything up. Used for the entire POWER_STM state.
 * ───────────────────────────────────────────────────────────────────────────*/

/* ── Clock gating ───────────────────────────────────────────────────────────*/
void PWR_BlockFastClocks(void);
void PWR_UnblockFastClocks(void);

/* ── Individual peripheral power gates ─────────────────────────────────────*/
void PWR_EnableI2C0(void);
void PWR_DisableI2C0(void);

void PWR_EnableI2C1(void);
void PWR_DisableI2C1(void);

void PWR_EnableUART0(void);
void PWR_DisableUART0(void);

void PWR_EnableSPI1(void);
void PWR_DisableSPI1(void);

/* ── Compound profile helpers ───────────────────────────────────────────────*/

/**
 * @brief Enter MINIMUM profile.
 *        Disables I2C_0, I2C_1, UART_0, SPI_1, DMA.
 *        Blocks fast clocks. RTC stays alive.
 */
void PWR_EnterMinimumProfile(void);

/**
 * @brief Enter MEASURE profile from MINIMUM.
 *        Unblocks fast clocks and enables I2C_0 only.
 *        Call at the start of a safety / charging check window.
 */
void PWR_EnterMeasureProfile(void);

/**
 * @brief Return to MINIMUM profile from MEASURE.
 *        Disables I2C_0 and re-blocks fast clocks.
 *        Call after I2C reads are complete.
 */
void PWR_ExitMeasureProfile(void);

/**
 * @brief Enter ACTIVE profile.
 *        Unblocks fast clocks, enables UART_0, I2C_0, SPI_1, DMA.
 *        Call at entry of POWER_STM state.
 */
void PWR_EnterActiveProfile(void);

/**
 * @brief Exit ACTIVE profile back to MINIMUM.
 *        Disables SPI_1, DMA, I2C_0, UART_0. Re-blocks fast clocks.
 *        Call when leaving POWER_STM state.
 */
void PWR_ExitActiveProfile(void);

void hall_init(void);
void gauge_init(void);

void RTC_EnablePrescaler(void);
void RTC_DisablePrescaler(void);

void PWR_EnableCoreInterrupts(void);
/* ─────────────────────────────────────────────────────────────────────────────
 * Flash saving funcitons
 * ───────────────────────────────────────────────────────────────────────────*/

void SM_LoadSTMConfig(void);
void SM_LoadCredentials(void);
void SM_LoadCharger(void);
void SM_LoadPeriod(void);

/* ── EEPROM Identifiers ──────────────────────────────────── */
typedef enum {
    /* Sentinels / flags */
    EEPROM_ID_CHARGER_CONFIGURED        = 1,
    EEPROM_ID_PERIOD_CONFIGURED         = 2,
    EEPROM_ID_STMCONFIG_CONFIGURED      = 3,

    /* Charger fields */
    EEPROM_ID_CHARGER_VREG              = 4,
    EEPROM_ID_CHARGER_ICHG              = 5,
    EEPROM_ID_CHARGER_IINDPM            = 6,
    EEPROM_ID_CHARGER_VINDPM            = 7,
    EEPROM_ID_CHARGER_VSYSMIN           = 8,
    EEPROM_ID_CHARGER_IPRECHG           = 9,
    EEPROM_ID_CHARGER_ITERM             = 10,

    /* Period fields */
    EEPROM_ID_WAKE_INTERVAL_MINUTES     = 11,

    /* STM config fields */
    EEPROM_ID_STM_CONN_MODE             = 12,
    EEPROM_ID_STM_LTE_COMM              = 13,
    EEPROM_ID_STM_LTE_BAUD              = 14,
    EEPROM_ID_STM_LTE_PROVIDER          = 15,
    EEPROM_ID_STM_CAM_RES               = 16,
    EEPROM_ID_STM_CAM_FPS               = 17,
    EEPROM_ID_STM_CAM_COMP              = 18,
    EEPROM_ID_STM_LOG_CARD              = 19,
    EEPROM_ID_STM_LOG_USART             = 20,

} SM_EEPROM_ID_t;

/* ── Public API ──────────────────────────────────────────── */
void SM_EEPROM_Init(void);
void SM_EEPROM_LoadAll(void);

void SM_EEPROM_SaveCharger(void);
void SM_EEPROM_SavePeriod(void);
void SM_EEPROM_SaveSTMConfig(void);

// ─────────────────────────────────────────────
// PWM Configuration
// ─────────────────────────────────────────────

typedef struct {
    GPTIMER_Regs *TIMER;
    uint8_t CC_INDEX;
    uint8_t is_complementary_output;
} PWM_Config;


// ─────────────────────────────────────────────
// LED Control State
// ─────────────────────────────────────────────

typedef struct {
    float set_current;   // currently set current in mA
} led_channel_t;

extern led_channel_t led_channel;       // single LED channel state
extern uint16_t global_led_voltage;     // currently set boost voltage in mV

// ─────────────────────────────────────────────
// Limits
// ─────────────────────────────────────────────

#define LED_HW_MAX_CURRENT_MA   2000     // hardware maximum current in mA
#define MAX_DUTY_CYCLES_LED     99      // number of entries in current LUT
#define MAX_DUTY_CYCLES_BOOST   35      // number of entries in voltage LUT

// ─────────────────────────────────────────────
// Initialisation
// ─────────────────────────────────────────────

/*
 * @brief Initialises PWM hardware and LED control state.
 *        Call once at startup before any other LED function.
 */
void LED_control_init(void);

// ─────────────────────────────────────────────
// Voltage Control
// ─────────────────────────────────────────────

/*
 * @brief Sets the LED boost converter output voltage.
 *        Can be called at any time to change the voltage.
 * @param voltage  Target voltage in mV (valid range ~3500 to ~11500)
 */
void LED_set_voltage(uint16_t voltage);

/*
 * @brief Returns the last voltage set via LED_set_voltage().
 * @return Voltage in mV
 */
uint16_t LED_get_voltage(void);

// ─────────────────────────────────────────────
// Current Control
// ─────────────────────────────────────────────

/*
 * @brief Sets the LED output current.
 *        Current is clamped to LED_HW_MAX_CURRENT_MA.
 * @param current  Target current in mA
 */
void LED_set_current(uint16_t current);

/*
 * @brief Returns the last current set via LED_set_current().
 * @return Current in mA
 */
uint16_t LED_get_current(void);

// ─────────────────────────────────────────────
// PWM Hardware Layer
// ─────────────────────────────────────────────

/*
 * @brief Writes a duty cycle value to a PWM channel.
 *        Handles VDD scaling and timer register write.
 * @param pwm_channel  Pointer to PWM channel config
 * @param duty_cycle   Duty cycle value (0-99 for LED channels, 0-399 for buck)
 */
void set_pwm_duty_cycle(const PWM_Config *pwm_channel, uint16_t duty_cycle);


//enable led boost converter
void enable_led_boost(void);

//disable led boost converter
void disable_led_boost(void);

void LED_flash_start(uint16_t on_ms);
void LED_flash_stop(void);

void PIR_interrupt(bool enable);
void PIR_Interrupt_PauseForI2C(void);
void PIR_Interrupt_ResumeAfterI2C(void);

uint16_t ZDP_ScanAddresses(I2C_Regs *targetBus);

#endif /* FUNC_HELPERS_H */