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

#endif /* FUNC_HELPERS_H */