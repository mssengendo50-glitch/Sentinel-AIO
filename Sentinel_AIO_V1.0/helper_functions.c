#include "helper_functions.h"
#include "ti_msp_dl_config.h"
#include "HAL/spi_master.h"
#include "HAL/uart.h"
#include "HAL/i2c.h"
#include "sm.h"
#include "ics/BQ25628/BQ25628_functions.h"
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

/* ═════════════════════════════════════════════════════════════════════════════
 * Load helpers
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