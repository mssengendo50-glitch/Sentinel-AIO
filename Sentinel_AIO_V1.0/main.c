#include "ti_msp_dl_config.h"
#include "HAL/i2c.h"
#include "functions.h"
#include "HAL/uart.h"
#include "ics/BQ25628/BQ25628_functions.h"
#include "ics/BQ27Z7/BQ27Z7_functions.h"
#include "HAL/spi_master.h"
#include "sm.h"
#include "helper_functions.h"


volatile bool bq_monitor_active    = false;
volatile bool hall_monitor_active  = false;
volatile bool gauge_monitor_active = false;
volatile bool rtc_minute_tick  = false;
volatile bool rtc_second_tick  = false;
volatile bool hall_wakeup_flag = false;
volatile bool stm_io2_flag     = false;
volatile uint32_t monitor_rate = 200; 
volatile uint32_t EEPROMEmulationState;  

void setupCLI(void) {
    CLI_RegisterCommand("help", cmd_help, "Show available commands");
    CLI_RegisterCommand("pwr",  cmd_pwr,  "Control power rails: 3v8, lora, lte, wifi, stm");
    CLI_RegisterCommand("i2cscan", cmd_i2cscan, "Scan I2C bus: i2cscan <0|1>");
    CLI_RegisterCommand("hall", cmd_hall, "Hall sensor: hall <pwr|status>");
    CLI_RegisterCommand("bq", cmd_bq, "BQ25628E charger control - type bq for full help");
    CLI_RegisterCommand("gauge",   cmd_gauge,   "BQ27Z746 gauge - type gauge for help");
    CLI_RegisterCommand("sm", cmd_sm, "State Machine control: status, start, stop");
}


int main(void)
{
    SYSCFG_DL_init(); 
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
        }else {
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


void GROUP1_IRQHandler(void) {
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        
        case EXTERNAL_INTERRUPT_GPIOA_INT_IIDX: 
            DL_GPIO_clearInterruptStatus(GPIOA, EXTERNAL_INTERRUPT_STM_MCU_IO2_PIN);
            stm_io2_flag = true;
            break;

        case EXTERNAL_INTERRUPT_GPIOB_INT_IIDX: 
            DL_GPIO_clearInterruptStatus(GPIOB, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
            hall_wakeup_flag = true;
            break;
    }
}
