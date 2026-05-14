#include "helper_functions.h"
#include "ti_msp_dl_config.h"
#include "HAL/spi_master.h"
#include "HAL/uart.h"
#include "HAL/i2c.h"
#include "sm.h"
#include "ics/BQ25628/BQ25628_functions.h"
#include "ti/driverlib/dl_timerg.h"
#include <stdint.h>
#include <stdlib.h>


/* ═════════════════════════════════════════════════════════════════════════════
 * Clock gating
 * ═══════════════════════════════════════════════════════════════════════════*/

void PWR_BlockFastClocks(void)
{
    DL_SYSCTL_blockAllAsyncFastClockRequests();
    DL_SYSCTL_disableSYSPLL();
}

void PWR_UnblockFastClocks(void)
{
    DL_SYSCTL_allowAllAsyncFastClockRequests();
}

/* ═════════════════════════════════════════════════════════════════════════════
 * I2C_0
 * ═══════════════════════════════════════════════════════════════════════════*/


void PWR_EnableI2C0(void)
{
    DL_I2C_reset(I2C_0_INST);
    DL_I2C_enablePower(I2C_0_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    SYSCFG_DL_I2C_0_init();
    i2c_init();
}

void PWR_DisableI2C0(void)
{
    DL_I2C_disableController(I2C_0_INST);
    DL_I2C_disablePower(I2C_0_INST);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * I2C_1
 * ═══════════════════════════════════════════════════════════════════════════*/

void PWR_EnableI2C1(void)
{
    DL_I2C_reset(I2C_1_INST);
    DL_I2C_enablePower(I2C_1_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    SYSCFG_DL_I2C_1_init();
    i2c_init();
}

void PWR_DisableI2C1(void)
{
    DL_I2C_disableController(I2C_1_INST);
    DL_I2C_disablePower(I2C_1_INST);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * UART_0
 * ═══════════════════════════════════════════════════════════════════════════*/

void PWR_EnableUART0(void)
{
    DL_UART_Main_reset(UART_0_INST);
    DL_UART_Main_enablePower(UART_0_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    SYSCFG_DL_UART_0_init();
    uart_init();
}

void PWR_DisableUART0(void)
{
    DL_UART_Main_disable(UART_0_INST);
    DL_UART_Main_disablePower(UART_0_INST);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * SPI_1 + DMA
 * ═══════════════════════════════════════════════════════════════════════════*/

void PWR_EnableSPI1(void)
{
    DL_SPI_reset(SPI_1_INST);
    DL_SPI_enablePower(SPI_1_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    SYSCFG_DL_SPI_1_init();
    SYSCFG_DL_DMA_init();
    spi_init();
}

void PWR_DisableSPI1(void)
{
    DL_SPI_disable(SPI_1_INST);
    DL_SPI_disablePower(SPI_1_INST);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Compound profile helpers
 * ═══════════════════════════════════════════════════════════════════════════*/

void PWR_EnterMinimumProfile(void)
{
    PWR_DisableSPI1();
    PWR_DisableI2C1();
}

void PWR_EnterMeasureProfile(void)
{
    PWR_EnableI2C0();
    PWR_UnblockFastClocks();
    PWR_EnableUART0();
    delay_cycles(3200);
}

void PWR_ExitMeasureProfile(void)
{
    PWR_DisableI2C0();
    PWR_DisableUART0();
    PWR_BlockFastClocks();
    delay_cycles(3200);
}

void PWR_EnterActiveProfile(void)
{
    PWR_EnableSPI1();
    delay_cycles(3200);
}

void PWR_ExitActiveProfile(void)
{
    PWR_DisableSPI1();
    delay_cycles(3200);
}

void hall_init(void) {
    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_HALL_3V_PIN);
    delay_cycles(1000);
    DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
}

void gauge_init(void) {
    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_GAUGE_EN_PIN);
    delay_cycles(320000);
    DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_GAUGE_EN_PIN);
}

void PWR_EnableCoreInterrupts(void)
{
    uart_init();
    i2c_init();
    spi_init();
    NVIC_EnableIRQ(RTC_INT_IRQn);
    NVIC_EnableIRQ(EXTERNAL_INTERRUPT_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(EXTERNAL_INTERRUPT_GPIOA_INT_IRQN);
}

void RTC_EnablePrescaler(void) {
    DL_RTC_enableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER1);
}

void RTC_DisablePrescaler(void) {
    DL_RTC_disableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER1);
}


void SM_LoadPeriod(void){
    sm_context.stm_wake_period.wake_interval_minutes = SM_SLEEP_WAKEUP_MINUTES;
    sm_context.stm_wake_period.wake_mode = 1;
    sm_context.wake_interval_configured = false;
}
void SM_LoadCharger(void){
        /* Charger defaults */
    sm_context.sm_charger_config = (SM_ChargerConfig_t){
        .vreg_mV     = BQ_INIT_VREG_MV,
        .ichg_mA     = BQ_INIT_ICHG_MA,
        .iindpm_mA   = BQ_INIT_IINDPM_MA,
        .vindpm_mV   = BQ_INIT_VINDPM_MV,
        .vsysmin_mV  = BQ_INIT_VSYSMIN_MV,
        .iprechg_mA  = BQ_INIT_IPRECHG_MA,
        .iterm_mA    = BQ_INIT_ITERM_MA
    };
    sm_context.charger_configured = false;
}

void SM_LoadSTMConfig(void)
{
    sm_context.stm_config = (SM_STMConfig_t){
        .connectivity = { .mode = 0 },
        .lte = {
            .communication    = 0,
            .baudrate_index   = 3,
            .network_provider = 0
        },
        .camera = {
            .resolution  = 2,
            .framerate   = 4,
            .compression = 1
        },
        .logging = {
            .log_to_card  = 0,
            .log_to_usart = 1
        }
    };
    sm_context.stm_config_received = false;
}

void SM_LoadCredentials(void){
    sm_context.stm_credentials = (SM_STMCredentials_t){
        .ap_ssid         = "KAMITECK",
        .ap_password     = "12345678",
        .device_name     = "AnfaEng",
        .device_password = "12345678"
    };
    sm_context.stm_credentials_received = false;
}


void SM_EEPROM_Init(void)
{
    uint32_t state = EEPROM_TypeB_init();
    if (state == EEPROM_EMULATION_INIT_OK) {
        uart_printf("[EEPROM] Init OK\n");
    } else if (state == EEPROM_EMULATION_INIT_OK_ALL_ERASE) {
        uart_printf("[EEPROM] Init OK - all erased (first boot)\n");
    } else if (state == EEPROM_EMULATION_INIT_OK_FORMAT_REPAIR) {
        uart_printf("[EEPROM] Init OK - format repaired\n");
    } else {
        uart_printf("[EEPROM] Init FAILED\n");
    }
}

void PIR_interrupt(bool enable) {
    if(enable){
        DL_GPIO_clearInterruptStatus(EXTERNAL_INTERRUPT_PIR_TRIGGER_PORT,
                                    EXTERNAL_INTERRUPT_PIR_TRIGGER_PIN);
        DL_GPIO_enableInterrupt(EXTERNAL_INTERRUPT_PIR_TRIGGER_PORT, EXTERNAL_INTERRUPT_PIR_TRIGGER_PIN);
    }else{
        DL_GPIO_disableInterrupt(EXTERNAL_INTERRUPT_PIR_TRIGGER_PORT, EXTERNAL_INTERRUPT_PIR_TRIGGER_PIN);
    }

}


/* ═════════════════════════════════════════════════════════════════════════════
 * EEPROMLoad helpers
 * ═══════════════════════════════════════════════════════════════════════════*/

static void SM_EEPROM_LoadCharger(void)
{
    /* Check sentinel */
    uint32_t configured = EEPROM_TypeB_readDataItem(EEPROM_ID_CHARGER_CONFIGURED);
    if (!gEEPROMTypeBSearchFlag || !configured) {
        uart_printf("[EEPROM] Charger: no saved config, using defaults\n");
        SM_LoadCharger();
        return;
    }

    /* Read each field, fall back to default value if any individual read fails */
    SM_LoadCharger();  /* load defaults first as a safety base */

    uint32_t val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_CHARGER_VREG);
    if (gEEPROMTypeBSearchFlag) sm_context.sm_charger_config.vreg_mV     = (uint16_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_CHARGER_ICHG);
    if (gEEPROMTypeBSearchFlag) sm_context.sm_charger_config.ichg_mA     = (uint16_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_CHARGER_IINDPM);
    if (gEEPROMTypeBSearchFlag) sm_context.sm_charger_config.iindpm_mA   = (uint16_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_CHARGER_VINDPM);
    if (gEEPROMTypeBSearchFlag) sm_context.sm_charger_config.vindpm_mV   = (uint16_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_CHARGER_VSYSMIN);
    if (gEEPROMTypeBSearchFlag) sm_context.sm_charger_config.vsysmin_mV  = (uint16_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_CHARGER_IPRECHG);
    if (gEEPROMTypeBSearchFlag) sm_context.sm_charger_config.iprechg_mA  = (uint16_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_CHARGER_ITERM);
    if (gEEPROMTypeBSearchFlag) sm_context.sm_charger_config.iterm_mA    = (uint16_t)val;

    sm_context.charger_configured = true;
    uart_printf("[EEPROM] Charger: config restored from flash\n");
}

static void SM_EEPROM_LoadPeriod(void)
{
    /* Check sentinel */
    uint32_t configured = EEPROM_TypeB_readDataItem(EEPROM_ID_PERIOD_CONFIGURED);
    if (!gEEPROMTypeBSearchFlag || !configured) {
        uart_printf("[EEPROM] Period: no saved config, using defaults\n");
        SM_LoadPeriod();
        return;
    }

    SM_LoadPeriod();  /* load defaults first as a safety base */

    uint32_t val = EEPROM_TypeB_readDataItem(EEPROM_ID_WAKE_INTERVAL_MINUTES);
    if (gEEPROMTypeBSearchFlag) {
        sm_context.stm_wake_period.wake_interval_minutes = (uint8_t)val;
    }

    sm_context.wake_interval_configured = true;
    uart_printf("[EEPROM] Period: config restored from flash\n");
}

static void SM_EEPROM_LoadSTMConfig(void)
{
    /* Check sentinel */
    uint32_t configured = EEPROM_TypeB_readDataItem(EEPROM_ID_STMCONFIG_CONFIGURED);
    if (!gEEPROMTypeBSearchFlag || !configured) {
        uart_printf("[EEPROM] STMConfig: no saved config, using defaults\n");
        SM_LoadSTMConfig();
        return;
    }

    SM_LoadSTMConfig();  /* load defaults first as a safety base */

    uint32_t val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_STM_CONN_MODE);
    if (gEEPROMTypeBSearchFlag) sm_context.stm_config.connectivity.mode        = (uint8_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_STM_LTE_COMM);
    if (gEEPROMTypeBSearchFlag) sm_context.stm_config.lte.communication         = (uint8_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_STM_LTE_BAUD);
    if (gEEPROMTypeBSearchFlag) sm_context.stm_config.lte.baudrate_index        = (uint8_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_STM_LTE_PROVIDER);
    if (gEEPROMTypeBSearchFlag) sm_context.stm_config.lte.network_provider      = (uint8_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_STM_CAM_RES);
    if (gEEPROMTypeBSearchFlag) sm_context.stm_config.camera.resolution         = (uint8_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_STM_CAM_FPS);
    if (gEEPROMTypeBSearchFlag) sm_context.stm_config.camera.framerate          = (uint8_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_STM_CAM_COMP);
    if (gEEPROMTypeBSearchFlag) sm_context.stm_config.camera.compression        = (uint8_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_STM_LOG_CARD);
    if (gEEPROMTypeBSearchFlag) sm_context.stm_config.logging.log_to_card       = (uint8_t)val;

    val = EEPROM_TypeB_readDataItem(EEPROM_ID_STM_LOG_USART);
    if (gEEPROMTypeBSearchFlag) sm_context.stm_config.logging.log_to_usart      = (uint8_t)val;

    sm_context.stm_config_received = true;
    uart_printf("[EEPROM] STMConfig: config restored from flash\n");
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Load All  — called once in SM_Init()
 * ═══════════════════════════════════════════════════════════════════════════*/

void SM_EEPROM_LoadAll(void)
{
    SM_EEPROM_LoadCharger();
    SM_EEPROM_LoadPeriod();
    SM_EEPROM_LoadSTMConfig();
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Save functions — each called immediately after its flag is set to true
 * ═══════════════════════════════════════════════════════════════════════════*/

void SM_EEPROM_SaveCharger(void)
{
    const SM_ChargerConfig_t *cfg = &sm_context.sm_charger_config;

    EEPROM_TypeB_write(EEPROM_ID_CHARGER_VREG,    cfg->vreg_mV);
    EEPROM_TypeB_write(EEPROM_ID_CHARGER_ICHG,    cfg->ichg_mA);
    EEPROM_TypeB_write(EEPROM_ID_CHARGER_IINDPM,  cfg->iindpm_mA);
    EEPROM_TypeB_write(EEPROM_ID_CHARGER_VINDPM,  cfg->vindpm_mV);
    EEPROM_TypeB_write(EEPROM_ID_CHARGER_VSYSMIN, cfg->vsysmin_mV);
    EEPROM_TypeB_write(EEPROM_ID_CHARGER_IPRECHG, cfg->iprechg_mA);
    EEPROM_TypeB_write(EEPROM_ID_CHARGER_ITERM,   cfg->iterm_mA);

    /* Write sentinel last — only marks config as valid once all fields are written */
    EEPROM_TypeB_write(EEPROM_ID_CHARGER_CONFIGURED, 1);

    uart_printf("[EEPROM] Charger config saved\n");
}

void SM_EEPROM_SavePeriod(void)
{
    EEPROM_TypeB_write(EEPROM_ID_WAKE_INTERVAL_MINUTES,
        sm_context.stm_wake_period.wake_interval_minutes);

    /* Write sentinel last */
    EEPROM_TypeB_write(EEPROM_ID_PERIOD_CONFIGURED, 1);

    uart_printf("[EEPROM] Period config saved\n");
}

void SM_EEPROM_SaveSTMConfig(void)
{
    const SM_STMConfig_t *cfg = &sm_context.stm_config;

    EEPROM_TypeB_write(EEPROM_ID_STM_CONN_MODE,     cfg->connectivity.mode);
    EEPROM_TypeB_write(EEPROM_ID_STM_LTE_COMM,      cfg->lte.communication);
    EEPROM_TypeB_write(EEPROM_ID_STM_LTE_BAUD,      cfg->lte.baudrate_index);
    EEPROM_TypeB_write(EEPROM_ID_STM_LTE_PROVIDER,  cfg->lte.network_provider);
    EEPROM_TypeB_write(EEPROM_ID_STM_CAM_RES,       cfg->camera.resolution);
    EEPROM_TypeB_write(EEPROM_ID_STM_CAM_FPS,       cfg->camera.framerate);
    EEPROM_TypeB_write(EEPROM_ID_STM_CAM_COMP,      cfg->camera.compression);
    EEPROM_TypeB_write(EEPROM_ID_STM_LOG_CARD,      cfg->logging.log_to_card);
    EEPROM_TypeB_write(EEPROM_ID_STM_LOG_USART,     cfg->logging.log_to_usart);

    /* Write sentinel last */
    EEPROM_TypeB_write(EEPROM_ID_STMCONFIG_CONFIGURED, 1);

    uart_printf("[EEPROM] STM config saved\n");
}


/* ═════════════════════════════════════════════════════════════════════════════
 * LED lights helpers
 * ═══════════════════════════════════════════════════════════════════════════*/

// ─────────────────────────────────────────────
// PWM Channel Definitions
// ─────────────────────────────────────────────

// Index 0: LED boost converter (voltage control)
// Index 1: LED1 current control
static const PWM_Config _pwm_outputs[3] = {
    {BOOST_CONTROL_INST, GPIO_BOOST_CONTROL_C0_IDX, 0},  // boost converter → voltage
    {BOOST_CONTROL_INST, GPIO_BOOST_CONTROL_C2_IDX, 0},  // LED1             → current
    {FLASH_CONTROL_INST, GPIO_FLASH_CONTROL_C1_IDX, 0},  // Flash LED         → current

};
// ─────────────────────────────────────────────
// Current Lookup Tables
// ─────────────────────────────────────────────

// Lookup Tables for Boost output voltage
uint8_t duty_cycles_boost[MAX_DUTY_CYCLES_BOOST] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35
};

uint16_t output_voltages_boost_mV[MAX_DUTY_CYCLES_BOOST] = {
    11330, 11070, 10880, 10570, 10330,
    10070, 9820, 9570, 9320, 9100,
    9070, 8830, 8580, 8310, 8060,
    7810, 7300, 7070, 6800, 6550,
    6540, 6290, 6030, 5780, 5580,
    5268, 5015, 4757, 4700, 4244,
    4050, 3995, 3994, 3734, 3490
};

// Lookup Tables for LED Output current
uint8_t duty_cycles_led[MAX_DUTY_CYCLES_LED] = {
    0, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
    51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
    61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
    71, 72, 73, 74, 75, 76, 77, 78, 79, 80,
    81, 82, 83, 84, 85, 86, 87, 88, 89, 90,
    91, 92, 93, 94, 95, 96, 97, 98, 99
};


uint16_t output_currents_led_mA[MAX_DUTY_CYCLES_LED] = {
    0, 16, 44, 74, 104, 132, 162, 192, 220, 250,
    280, 310, 338, 368, 396, 426, 454, 486, 514, 542,
    572, 602, 632, 662, 692, 722, 752, 782, 812, 842,
    872, 902, 932, 962, 992, 1022, 1052, 1082, 1112, 1142,
    1172, 1202, 1232, 1262, 1292, 1322, 1352, 1382, 1412, 1442,
    1472, 1502, 1532, 1562, 1592, 1622, 1652, 1682, 1712, 1742,
    1772, 1802, 1832, 1862, 1892, 1922, 1952, 1982, 2012, 2042,
    2072, 2102, 2132, 2162, 2192, 2222, 2252, 2282, 2312, 2342,
    2372, 2402, 2432, 2462, 2492, 2522, 2552, 2582, 2612, 2642,
    2672, 2702, 2732, 2762, 2792, 2822, 2852, 2882, 2912
};


// ─────────────────────────────────────────────
// LED State
// ─────────────────────────────────────────────

led_channel_t led_channel = { .set_current = 0 };
uint16_t global_led_voltage = 0;

// ─────────────────────────────────────────────
// Internal Helpers
// ─────────────────────────────────────────────

// Binary search on an ascending sorted LUT.
// Returns the index of the entry closest to the requested value.
static uint16_t _binary_search_ascending(uint16_t value, const uint16_t *LUT, uint16_t table_size) {
    int16_t left = 0, right = table_size - 1;
    int16_t closest_index = 0;

    while (left <= right) {
        int16_t mid = left + (right - left) / 2;

        uint16_t diff_mid     = (uint16_t)abs((int16_t)LUT[mid]           - (int16_t)value);
        uint16_t diff_closest = (uint16_t)abs((int16_t)LUT[closest_index] - (int16_t)value);

        if (diff_mid < diff_closest) closest_index = mid;

        if      (LUT[mid] == value) return mid;
        else if (LUT[mid] <  value) left  = mid + 1;
        else                        right = mid - 1;
    }

    return closest_index;
}

// Binary search on a descending sorted LUT.
// Returns the index of the entry closest to the requested value.
// Used for the boost voltage table which goes high to low.
static uint16_t _binary_search_descending(uint16_t value, const uint16_t *LUT, uint16_t table_size) {
    int16_t left = 0, right = table_size - 1;
    int16_t closest_index = 0;

    while (left <= right) {
        int16_t mid = left + (right - left) / 2;

        uint16_t diff_mid     = (uint16_t)abs((int16_t)LUT[mid]           - (int16_t)value);
        uint16_t diff_closest = (uint16_t)abs((int16_t)LUT[closest_index] - (int16_t)value);

        if (diff_mid < diff_closest) closest_index = mid;

        if      (LUT[mid] == value) return mid;
        else if (LUT[mid] >  value) left  = mid + 1;  // descending: go right if too high
        else                        right = mid - 1;
    }

    return closest_index;
}

// ─────────────────────────────────────────────
// PWM Hardware Layer
// ─────────────────────────────────────────────

void set_pwm_duty_cycle(const PWM_Config *pwm_channel, uint16_t duty_cycle) {
    if (pwm_channel == &_pwm_outputs[2]) {
        DL_TimerA_stopCounter(pwm_channel->TIMER);
        DL_TimerA_setCaptureCompareValue(
            pwm_channel->TIMER,
            duty_cycle,
            pwm_channel->CC_INDEX
        );
        DL_TimerA_startCounter(pwm_channel->TIMER);
    } else {
        DL_TimerA_stopCounter(pwm_channel->TIMER);
        DL_TimerA_setCaptureCompareValue(
            pwm_channel->TIMER,
            100 - duty_cycle,
            pwm_channel->CC_INDEX
        );
        DL_TimerA_startCounter(pwm_channel->TIMER);
    }
}

// ─────────────────────────────────────────────
// Voltage Control
// ─────────────────────────────────────────────

void LED_set_voltage(uint16_t voltage) {
    const uint16_t LED_VMAX = 11540;
    const uint16_t v_d = 90;
    
    uint16_t duty_cycle = (LED_VMAX - voltage) / v_d;

    if(duty_cycle > 99) duty_cycle = 99;
    if(duty_cycle < 1)  duty_cycle = 1;

    global_led_voltage = voltage;
    set_pwm_duty_cycle(&_pwm_outputs[0], duty_cycle);
}

uint16_t LED_get_voltage(void) {
    return global_led_voltage;
}

// ─────────────────────────────────────────────
// Current Control
// ─────────────────────────────────────────────

void LED_set_current(uint16_t current) {
    // Clamp to hardware maximum
    if (current > LED_HW_MAX_CURRENT_MA) current = LED_HW_MAX_CURRENT_MA;

    // Find the closest matching current in the LUT
    uint16_t index = _binary_search_ascending(current, output_currents_led_mA, MAX_DUTY_CYCLES_LED);

    // Record the actual set current (what the LUT entry gives, not the raw request)
    led_channel.set_current = output_currents_led_mA[index];

    // Apply duty cycle for LED1 channel
    set_pwm_duty_cycle(&_pwm_outputs[1], duty_cycles_led[index]);
}

uint16_t LED_get_current(void) {
    return (uint16_t)led_channel.set_current;
}

// ─────────────────────────────────────────────
// Initialisation
// ─────────────────────────────────────────────

void LED_control_init(void) {
    led_channel.set_current = 0;
    global_led_voltage      = 0;

    // Start with both PWM outputs at zero / off
    LED_set_current(0);
    LED_set_voltage(11330);
}


void enable_led_boost(void){
    DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_IR_ENABLE_PIN);
};


void disable_led_boost(void){
    DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_IR_ENABLE_PIN);
};

void LED_flash_start(uint16_t on_ms) {
    if (on_ms < 1)  on_ms = 1;
    if (on_ms > 50) on_ms = 50;

    uint16_t ticks = on_ms * 500;
    set_pwm_duty_cycle(&_pwm_outputs[2], ticks);
}

void LED_flash_stop(void) {
    DL_TimerA_stopCounter(FLASH_CONTROL_INST);
}


/* ═════════════════════════════════════════════════════════════════════════════
 * PIR INIT helpers
 * ═══════════════════════════════════════════════════════════════════════════*/
#include "ics/ZILOG/ZDP323B.h"

uint16_t ZDP_ScanAddresses(I2C_Regs *targetBus) {
    uint16_t addrs[] = {0x301, 0x302, 0x303, 0x304};
    for (int i = 0; i < 4; i++) {
        if (I2C_TryAddress10(targetBus, addrs[i])) {
            return addrs[i];
        }
    }
    return 0;
}
