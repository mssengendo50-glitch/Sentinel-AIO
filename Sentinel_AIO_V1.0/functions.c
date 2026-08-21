#include "ti_msp_dl_config.h"
#include "HAL/uart.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>          /* log1p() - `ltr predict` feeds the model directly */
#include "HAL/i2c.h"
#include "ics/BQ25628/BQ25628_functions.h"
#include "HAL/spi_master.h"
#include "ics/BQ27Z7/BQ27Z7_functions.h"
#include "sm.h"
#include "ics/ZILOG/ZDP323B.h"
#include "ics/LTR329/LTR329.h"
#include "ics/LIS3DH/LIS3DH.h"
#include "ics/IMX335/IMX335.h"
#include "HAL/ticks.h"
#include "helper_functions.h"
extern volatile bool gauge_monitor_active;
extern volatile bool bq_monitor_active; 
extern volatile bool hall_monitor_active;
extern volatile uint32_t monitor_rate;
extern volatile bool pir_monitor_active;
extern volatile bool lis_monitor_active;
extern volatile bool ltr_monitor_active;
extern volatile bool ltr_model_monitor_active;

void Run_Legacy_Monitors(char* processingBuffer) {
    if (hall_monitor_active) {
        delay_cycles(monitor_rate * 32000); // Fixed 200ms delay
        
        uint32_t pin_state = DL_GPIO_readPins(EXTERNAL_INTERRUPT_CHARGER_INT_PORT, EXTERNAL_INTERRUPT_SETUP_INT_PIN);
        
        if (pin_state) {
            uart_printf("SETUP_INT: HIGH (1)\n");
        } else {
            uart_printf("SETUP_INT: LOW (0)\n");
        }
        
        // Check if user wants to stop
        if (data_received) {
            hall_monitor_active = false;
            
            get_UART_buffer(processingBuffer);
            uart_printf("Hall monitoring stopped\n");
        }
    }
    if (bq_monitor_active) {
        delay_cycles(monitor_rate * 32000); // 200 ms refresh

        uint32_t charger_int = DL_GPIO_readPins(EXTERNAL_INTERRUPT_CHARGER_INT_PORT, EXTERNAL_INTERRUPT_CHARGER_INT_PIN); 

        BQ25628E_UpdateTelemetry();
        uint8_t stat1 = BQ25628E_ReadReg8(BQ25628E_REG_STAT1);
        uint8_t chg_stat = (stat1 >> 3) & 0x03;

        const char* desc;
        switch (chg_stat) {
            case 0: desc = "Not Charging / Terminated"; break;
            case 1: desc = "Pre/Trickle/Fast (CC)"; break;
            case 2: desc = "Taper (CV)"; break;
            case 3: desc = "Top-Off"; break;
            default: desc = "Unknown";
        }

        uart_printf("=== BQ25628E MONITOR (200ms) ===\n");
        uart_printf("CHARGER_INT : %s\n", charger_int ? "HIGH" : "LOW");
        uart_printf("Charging Status : %s  (CHG_STAT[4:3] = 0b%02b)\n", desc, chg_stat);
        uart_printf("VBUS:%4dmV VBAT:%4dmV VSYS:%4dmV IBUS:%4dmA IBAT:%4dmA TBAT:%3.1fC TDIE:%3dC\n",
                    BQ25628E_Get_VBUS_mV(),
                    BQ25628E_Get_VBAT_mV(), 
                    BQ25628E_Get_VSYS_mV(),
                    BQ25628E_Get_IBUS_mA(), 
                    BQ25628E_Get_IBAT_mA(),
                    BQ25628E_Get_TBAT_C(), // Battery NTC temperature
                    BQ25628E_Get_TDIE_C()); // Charger chip internal temperature
        uart_printf("ChgFlag0:0x%02X  FaultFlag0:0x%02X\n",
                    BQ25628E_ReadReg8(BQ25628E_REG_CHG_FLAG0),
                    BQ25628E_ReadReg8(BQ25628E_REG_FAULT_FLAG0));

        if (data_received) { 
            bq_monitor_active = false;
            get_UART_buffer(processingBuffer);
            uart_printf("Monitor stopped\n");
        }
    }
    if (gauge_monitor_active) {
        delay_cycles(monitor_rate * 32000);

        BQ27Z746_UpdateTelemetry(I2C_0_INST);

        uint16_t tte = BQ27Z746_Get_TimeToEmpty_min();
        uint16_t ttf = BQ27Z746_Get_TimeToFull_min();

        /* Determine a one-word state string from cached BatteryStatus */
        const char *state;
        if      (BQ27Z746_IsDischarging())     state = "DISCHARGING";
        else if (BQ27Z746_IsFullyCharged())    state = "FULLY CHARGED";
        else if (BQ27Z746_IsFullyDischarged()) state = "FULLY DISCHARGED";
        else                                   state = "CHARGING";

        uart_printf("=== BQ27Z746 MONITOR (200ms) ===\n");
        uart_printf("State  : %s\n", state);
        uart_printf("SOC    : %3d %%   SoH: %3d %%   Cycles: %d\n",
                    BQ27Z746_Get_SOC_pct(),
                    BQ27Z746_Get_StateOfHealth_pct(),
                    BQ27Z746_Get_CycleCount());
        uart_printf("VBAT   : %4d mV\n", BQ27Z746_Get_Voltage_mV());
        uart_printf("IBAT   : %5d mA   AvgI: %5d mA\n",
                    BQ27Z746_Get_Current_mA(),
                    BQ27Z746_Get_AvgCurrent_mA());
        uart_printf("AvgPwr : %5d mW\n", BQ27Z746_Get_AvgPower_mW());
        uart_printf("RemCap : %4d mAh  FullCap: %4d mAh\n",
                    BQ27Z746_Get_RemainingCap_mAh(),
                    BQ27Z746_Get_FullChargeCap_mAh());
        uart_printf("Temp   : %3d C   InternalTemp: %3d C\n",
                    BQ27Z746_Get_Temperature_C(),
                    BQ27Z746_Get_InternalTemp_C());

        /* TTE / TTF with 0xFFFF guard */
        uart_printf("TTE    : ");
        if (tte == 0xFFFFu) uart_printf("  ---");
        else                uart_printf("%4d min", tte);

        uart_printf("   TTF: ");
        if (ttf == 0xFFFFu) uart_printf("  ---\n");
        else                uart_printf("%4d min\n", ttf);

        uart_printf("Status : 0x%04X\n", BQ27Z746_Get_BatteryStatus());
        uart_printf("--------------------------------\n");

        if (data_received) {
            gauge_monitor_active = false;
            get_UART_buffer(processingBuffer);
            uart_printf("Gauge monitor stopped\n");
        }
    }
    if (pir_monitor_active) {

        // Read Peak Hold from sensor
        int16_t peak = 0;
        I2C_Status st = ZDP323B_ReadPeakHold(gPIR.i2c, gPIR.dev_addr, &peak);

        if (st != I2C_SUCCESS) {
            uart_printf("[PIR] Read error during monitor\n");
            pir_monitor_active = false;
        } else {
            // Check and clear motion flag atomically
            bool motion = gPIR.motion_detected;
            if (motion) gPIR.motion_detected = false;

            uart_printf("[PIR] Peak: %5d  Threshold: ±%4d  Motion: %s\n",
                        peak,
                        gPIR.armed_cfg.threshold * 8,
                        motion ? "DETECTED" : "-");
        }
        if (data_received) {
            pir_monitor_active = false;
            get_UART_buffer(processingBuffer);
            uart_printf("[PIR] Monitor stopped\n");
        }
            pir_monitor_active = false;
            PIR_interrupt(true);
    }
    if (ltr_monitor_active) {
        delay_cycles(monitor_rate * 32000); // 200ms

        uint16_t ch0, ch1;
        if (LTR329_ReadData(&ch0, &ch1)) {
            float lux = LTR329_CalculateLux(ch0, ch1);
            uart_printf("[LTR] CH0: %5u  CH1: %5u  Lux: %7.2f\n", ch0, ch1, lux);
        } else {
            uart_printf("[LTR] Read error\n");
            ltr_monitor_active = false;
        }

        if (data_received) {
            ltr_monitor_active = false;
            get_UART_buffer(processingBuffer);
            uart_printf("[LTR] Monitor stopped\n");
        }
    }
    /* Model monitor. NOTE: on the branch this was ported from, `ltr
     * model_monitor` set its flag and nothing ever read it - the command
     * printed a CSV header and then sat silent. This block is the missing
     * consumer, so the switch does what its help text says. Slower than the
     * raw LTR monitor (500 ms) because the point is to watch predictions
     * track changing light, not to sample fast. */
    if (ltr_model_monitor_active) {
        delay_cycles(500 * 32000); // 500ms

        uint16_t ch0, ch1;
        if (LTR329_ReadData(&ch0, &ch1)) {
            LTR329_PrintPredictedExposureGain(LTR329_CalculateLux(ch0, ch1));
        } else {
            uart_printf("[LTR] Read error\n");
            ltr_model_monitor_active = false;
        }

        if (data_received) {
            ltr_model_monitor_active = false;
            get_UART_buffer(processingBuffer);
            uart_printf("[LTR] Model monitor stopped\n");
        }
    }
    if (lis_monitor_active) {
        delay_cycles(monitor_rate * 32000); // 200ms

        float x, y, z;
        if (LIS3DH_ReadMg(&x, &y, &z)) {
            uart_printf("[LIS] X: %8.2f  Y: %8.2f  Z: %8.2f mg\n", x, y, z);
        } else {
            uart_printf("[LIS] Read error\n");
            lis_monitor_active = false;
        }

        if (data_received) {
            lis_monitor_active = false;
            get_UART_buffer(processingBuffer);
            uart_printf("[LIS] Monitor stopped\n");
        }
    }
}

void cmd_sm(char *args) {
    char *tokens[1];
    int tokenCount = CLI_Tokenize(args, tokens, 1);

    if (tokenCount == 0) {
        uart_printf("SM Control CLI\n"
                    "  sm status  - print current state\n"
                    "  sm start   - resume state machine\n"
                    "  sm stop    - pause state machine\n"
                    "  sm timing  - last STM32 power-on, stage by stage\n");
        return;
    }

    char *sub = tokens[0];

if (strcmp(sub, "timing") == 0) {
    SM_PrintAeTiming();
    return;
}
else if (strcmp(sub, "status") == 0) {
    uart_printf("=== State Machine ===\n");
    uart_printf("  State         : %s -> %s\n",
        sm_context.previous == SM_STATE_INIT           ? "INIT"           :
        sm_context.previous == SM_STATE_CHARGING       ? "CHARGING"       :
        sm_context.previous == SM_STATE_POWER_STM      ? "POWER_STM"      :
        sm_context.previous == SM_STATE_IDLE           ? "IDLE"           :
        sm_context.previous == SM_STATE_CRITICAL_FAULT ? "CRITICAL_FAULT" : "UNKNOWN",
        SM_GetStateString());
    const char* wake_reason_str = "normal";
    if (sm_context.wake_reason == SM_WAKE_SETUP) wake_reason_str = "setup";
    else if (sm_context.wake_reason == SM_WAKE_PIR) wake_reason_str = "pir";
    uart_printf("  Wake Reason   : %s\n", wake_reason_str);
    uart_printf("  Paused        : %s\n", sm_context.sm_paused ? "YES" : "NO");
    uart_printf("  Minute Counter: %lu\n", sm_context.minute_counter);
    uart_printf("  Second Counter: %lu\n", sm_context.second_counter);

    if (sm_context.current == SM_STATE_CRITICAL_FAULT) {
        const char* fault_str = "UNKNOWN";
        switch (sm_context.fault_source) {
            case SM_FAULT_I2C_BUS:     fault_str = "I2C_BUS";    break;
            case SM_FAULT_GAUGE:       fault_str = "GAUGE";       break;
            case SM_FAULT_CHARGER:     fault_str = "CHARGER";     break;
            case SM_FAULT_INIT_FAILED: fault_str = "INIT_FAILED"; break;
            default: break;
        }
        uart_printf("  Fault Source  : %s\n", fault_str);
    }

    uart_printf("\n=== Charger Config ===\n");
    uart_printf("  VREG          : %d mV\n", sm_context.sm_charger_config.vreg_mV);
    uart_printf("  ICHG          : %d mA\n", sm_context.sm_charger_config.ichg_mA);
    uart_printf("  IINDPM        : %d mA\n", sm_context.sm_charger_config.iindpm_mA);
    uart_printf("  VINDPM        : %d mV\n", sm_context.sm_charger_config.vindpm_mV);
    uart_printf("  Configured    : %s\n", sm_context.charger_configured ? "YES (STM32)" : "NO (defaults)");

    uart_printf("\n=== RTC ===\n");
    RTC_GetTime(&sm_context.sm_rtc_config);
    uart_printf("  Time          : %02d:%02d:%02d\n",
        sm_context.sm_rtc_config.hour,
        sm_context.sm_rtc_config.minute,
        sm_context.sm_rtc_config.second);
    uart_printf("  Date          : %02d/%02d/%04d\n",
        sm_context.sm_rtc_config.day,
        sm_context.sm_rtc_config.month,
        sm_context.sm_rtc_config.year);
    uart_printf("  Wake Interval : %d min\n", sm_context.stm_wake_period.wake_interval_minutes);
    uart_printf("\n=== STM32 Session ===\n");
    uart_printf("  First Boot         : %s\n", sm_context.first_boot ? "YES" : "NO");
    uart_printf("  Total Wakes        : %lu\n", (unsigned long)sm_context.total_wakes);
    uart_printf("  Inactivity Timeouts: %lu\n", (unsigned long)sm_context.inactivity_timeouts);
    uart_printf("  Last Periodic Wake : %lu min\n", sm_context.last_stm_periodic_minute);
    uart_printf("  Critical Msg Sent  : %s\n", sm_context.critical_msg_sent ? "YES" : "NO");

    uart_printf("\n=== STM Config ===\n");
    uart_printf("  Source        : %s\n", sm_context.stm_config_received ? "STM32 (live)" : "defaults");
    uart_printf("  Connectivity  : %s\n", sm_context.stm_config.connectivity.mode == 0 ? "LTE" : "WiFi");
    uart_printf("  LTE Comms     : %s\n", sm_context.stm_config.lte.communication == 0 ? "USART" : "USB");
    uart_printf("  LTE Baud Idx  : %d\n", sm_context.stm_config.lte.baudrate_index);
    uart_printf("  LTE Provider  : %s\n", sm_context.stm_config.lte.network_provider == 0 ? "Roaming" : "Local");
    uart_printf("  Cam Res       : %d\n", sm_context.stm_config.camera.resolution);
    uart_printf("  Cam FPS       : %d\n", sm_context.stm_config.camera.framerate);
    uart_printf("  Cam Compress  : %d\n", sm_context.stm_config.camera.compression);
    uart_printf("  Log to Card   : %s\n", sm_context.stm_config.logging.log_to_card  ? "YES" : "NO");
    uart_printf("  Log to USART  : %s\n", sm_context.stm_config.logging.log_to_usart ? "YES" : "NO");

    uart_printf("\n=== STM Credentials ===\n");
    uart_printf("  Source        : %s\n", sm_context.stm_credentials_received ? "STM32 (live)" : "defaults");
    uart_printf("  AP SSID       : %s\n", sm_context.stm_credentials.ap_ssid);
    uart_printf("  Device Name   : %s\n", sm_context.stm_credentials.device_name);
}
    else if (strcmp(sub, "start") == 0) {
        sm_context.sm_paused = false;
        uart_printf("State machine RESUMED\n");
    }
    else if (strcmp(sub, "stop") == 0) {
        sm_context.sm_paused = true;
        uart_printf("State machine PAUSED\n");
    }
    else {
        uart_printf("Unknown sm sub-command.\n");
    }
}


void cmd_pwr(char *args) {
    char *tokens[2];
    int tokenCount = CLI_Tokenize(args, tokens, 2);

    if (tokenCount < 2) {
        uart_printf("Usage: pwr <rail_name> <1|0>\n");
        return;
    }

    char *rail = tokens[0];
    int state = atoi(tokens[1]);

    if (strcmp(rail, "3v8") == 0) {
        if(state) DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN);
        else      DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_EN3V8_PIN);
    } 
    else if (strcmp(rail, "lte") == 0) {
        if(state) DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_MCU_LTE_PON_PIN);
        else      DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_MCU_LTE_PON_PIN);
    }
    else if (strcmp(rail, "wifi") == 0) {
        if(state) DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_MCU_WIFI_PON_PIN);
        else      DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_MCU_WIFI_PON_PIN);
    }
    else if (strcmp(rail, "stm") == 0) {
        if(state) DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
        else      DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_STM_PON_PIN);
    }
    else {
        uart_printf("Unknown rail: %s\n", rail);
        return;
    }

    uart_printf("Power %s %s\n", rail, state ? "ENABLED" : "DISABLED");
}

void cmd_i2cscan(char *args) {
    char *tokens[1];
    int tokenCount = CLI_Tokenize(args, tokens, 1);
    
    I2C_Regs *targetBus;
    int busNum = (tokenCount > 0) ? atoi(tokens[0]) : 0; // Default to Bus 0

    if (busNum == 0)      targetBus = I2C_0_INST;
    else if (busNum == 1) targetBus = I2C_1_INST;
    else {
        uart_printf("Invalid bus. Use 0 or 1.\n");
        return;
    }

    uart_printf("Scanning I2C Bus %d...\n", busNum);
    uint8_t foundCount = 0;

    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        if (I2C_TryAddress(targetBus, addr)) {
            uart_printf("  Found device at 0x%02X\n", addr);
            foundCount++;
        }
    }

    if (foundCount == 0) {
        uart_printf("No devices found.\n");
    } else {
        uart_printf("Scan complete. %d device(s) found.\n", foundCount);
    }
}


void cmd_hall(char *args) {
    char *tokens[2];
    int tokenCount = CLI_Tokenize(args, tokens, 2);

    if (tokenCount < 1) {
        uart_printf("Usage: hall <pwr|status> [value]\n");
        uart_printf("  hall pwr 1    - Enable 3V_HALL power\n");
        uart_printf("  hall pwr 0    - Disable 3V_HALL power\n");
        uart_printf("  hall status   - Monitor SETUP_INT at 200ms (press any key to stop)\n");
        return;
    }

    char *subcmd = tokens[0];
    
    if (strcmp(subcmd, "pwr") == 0) {
        if (tokenCount < 2) {
            uart_printf("Usage: hall pwr <1|0>\n");
            return;
        }
        int state = atoi(tokens[1]);
        
        if (state) {
            DL_GPIO_setPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_HALL_3V_PIN);
            uart_printf("3V_HALL power ENABLED\n");
        } else {
            DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTA_PORT, DIGITAL_OUTPUT_PORTA_HALL_3V_PIN);
            uart_printf("3V_HALL power DISABLED\n");
        }
    }
    else if (strcmp(subcmd, "status") == 0) {
        extern volatile bool hall_monitor_active;
        
        if (hall_monitor_active) {
            // If already monitoring, stop it
            uart_printf("SETUP_INT monitoring stopped\n");
            hall_monitor_active = false;
        } else {
            // Start monitoring at fixed 200ms rate
            uart_printf("Monitoring SETUP_INT at 200ms rate (press any key to stop)\n");
            hall_monitor_active = true;
        }
    }
    else {
        uart_printf("Unknown subcommand: %s\n", subcmd);
    }
}


void cmd_bq(char *args) {
    char *tokens[4];
    int tokenCount = CLI_Tokenize(args, tokens, 4);

    if (tokenCount == 0) {
        uart_printf("BQ25628E Bring-up CLI\n"
                    "  bq init             - full charger init\n"
                    "  bq dump             - read all key registers\n"
                    "  bq enable           - start charging (EN_CHG + CE=LOW)\n"
                    "  bq disable          - stop charging (EN_CHG=0 + CE=HIGH)\n"
                    "  bq monitor          - 200ms status + flag monitor\n"
                    "  bq stop             - stop monitor\n" );
        return;
    }

    char *sub = tokens[0];

/* Charger initialization */
    if (strcmp(sub, "init") == 0) {
        // Pre-check device presence
        uart_printf("Checking for BQ25628E...\n");
        if (!I2C_TryAddress(I2C_0_INST, BQ25628E_I2C_ADDR)) {
            uart_printf("ERROR: Device not found at 0x%02X\n", BQ25628E_I2C_ADDR);
            uart_printf("Run 'i2cscan 0' to see available devices\n");
            return;
        }
        uart_printf("Device found! Initializing...\n");
    if (BQ25628E_Init_Default()) {
        uart_printf("Charger initialized successfully\n");
    } else {
        uart_printf("ERROR: Charger initialization failed\n");
    }
    }


    /* Read ALL configuration registers */
    else if (strcmp(sub, "dump") == 0) {
        uart_printf("=== BQ25628E Full Register Dump ===\n");
        uart_printf("ICHG   0x02: 0x%04X\n", BQ25628E_ReadReg16(0x02));
        uart_printf("VREG   0x04: 0x%04X\n", BQ25628E_ReadReg16(0x04));
        uart_printf("IINDPM 0x06: 0x%04X\n", BQ25628E_ReadReg16(0x06));
        uart_printf("VINDPM 0x08: 0x%04X\n", BQ25628E_ReadReg16(0x08));
        uart_printf("VSYSMIN0x0E: 0x%04X\n", BQ25628E_ReadReg16(0x0E));
        uart_printf("CTRL0  0x16: 0x%02X\n", BQ25628E_ReadReg8(0x16));
        uart_printf("CTRL1  0x17: 0x%02X\n", BQ25628E_ReadReg8(0x17));
        uart_printf("CTRL3  0x19: 0x%02X\n", BQ25628E_ReadReg8(0x19));
        uart_printf("NTC0   0x1A: 0x%02X\n", BQ25628E_ReadReg8(0x1A));
        uart_printf("STAT0  0x1D: 0x%02X\n", BQ25628E_ReadReg8(0x1D));
        uart_printf("STAT1  0x1E: 0x%02X  [CHG_STAT[4:3]=0b%02b]\n",
                    BQ25628E_ReadReg8(0x1E), (BQ25628E_ReadReg8(0x1E)>>3)&0x03);
        uart_printf("CHG_FLAG0 0x20: 0x%02X\n", BQ25628E_ReadReg8(0x20));
        uart_printf("FAULT_FLAG0 0x22: 0x%02X\n", BQ25628E_ReadReg8(0x22));
    }

    /* Enable Charging */
    else if (strcmp(sub, "enable") == 0) {
        // BQ25628E_Set_ChargerEnable(true);
        DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN); 
        uart_printf("Charging STARTED (EN_CHG=1 + CE=LOW)\n");
    }

    /* Disable Charging */
    else if (strcmp(sub, "disable") == 0) {
        // BQ25628E_Set_ChargerEnable(false);
        DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_CHARGER_EN_PIN); 
        uart_printf("Charging STOPPED (EN_CHG=0 + CE=HIGH)\n");
    }

    /* Monitor charging status and charger flag */
    else if (strcmp(sub, "monitor") == 0) {
        bq_monitor_active = true;
        uart_printf("Monitor started  — type any command to stop\n");
    }

    /* Stop monitoring */
    else if (strcmp(sub, "stop") == 0) {
        bq_monitor_active = false;
        uart_printf("Monitor stopped\n");
    }

    else {
        uart_printf("Unknown bq sub-command. Type 'bq' for help.\n");
    }
}


static void print_battery_status(uint16_t status)
{
    uart_printf("BatteryStatus: 0x%04X\n", status);
    uart_printf("  FC  (Fully Charged)    : %s\n", (status & BQ27Z746_STATUS_FC)   ? "YES" : "no");
    uart_printf("  FD  (Fully Discharged) : %s\n", (status & BQ27Z746_STATUS_FD)   ? "YES" : "no");
    uart_printf("  DSG (Discharging)      : %s\n", (status & BQ27Z746_STATUS_DSG)  ? "YES" : "no");
    uart_printf("  INIT (Initializing)    : %s\n", (status & BQ27Z746_STATUS_INIT) ? "YES" : "no");
    uart_printf("  RCA (RemainingCap Alm) : %s\n", (status & BQ27Z746_STATUS_RCA)  ? "YES" : "no");
    uart_printf("  TDA (TermDischarge Alm): %s\n", (status & BQ27Z746_STATUS_TDA)  ? "YES" : "no");
}

static void print_time(uint16_t minutes)
{
    if (minutes == 0xFFFFu)
        uart_printf("  ---");
    else
        uart_printf("%4dmin", minutes);
}

/* ================================================================
 * cmd_gauge
 * ================================================================ */
void cmd_gauge(char *args)
{
    char *tokens[4];
    int tokenCount = CLI_Tokenize(args, tokens, 4);

    if (tokenCount == 0) {
        uart_printf("BQ27Z746 Gauge CLI\n"
            "  gauge on <1|0>       - Pulls ENAB_N low/high\n"
            "  gauge init           - verify comms, confirm device type\n"
            "  gauge dump           - read all telemetry registers\n"
            "  gauge status         - decode BatteryStatus bits\n"
            "  gauge info           - device type, FW version, ChemID\n"
            "  gauge monitor        - 200ms live telemetry\n"
            "  gauge stop           - stop monitor\n"
            "\n");
        return;
    }

    char *sub = tokens[0];

    if (strcmp(sub, "on") == 0) {
        if (tokenCount < 2) {
            uart_printf("Usage: gauge on <1|0>\n");
            return;
        }
        int state = atoi(tokens[1]);
        if (state) {
            DL_GPIO_setPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_GAUGE_EN_PIN);
            uart_printf("Gauge ENAB_N Enabled\n");
        } else {
            DL_GPIO_clearPins(DIGITAL_OUTPUT_PORTB_PORT, DIGITAL_OUTPUT_PORTB_GAUGE_EN_PIN);
            uart_printf("Gauge ENAB_N disabled\n");
        }
    }

    else if (strcmp(sub, "init") == 0) {
        uart_printf("Checking for BQ27Z746 on I2C0...\n");

        if (!I2C_TryAddress(I2C_0_INST, GAUGE_I2C_ADDR)) {
            uart_printf("ERROR: No device found at 0x%02X\n", GAUGE_I2C_ADDR);
            uart_printf("Run 'i2cscan 0' to check what is on the bus\n");
            return;
        }

        if (!BQ27Z746_Init(I2C_0_INST)) {
            uart_printf("ERROR: Init failed — failed init for (TS)\n");
            return;
        }

        uart_printf("BQ27Z746 found and confirmed\n");

        uint16_t fw = 0u;
        if (BQ27Z746_GetFirmwareVersion(I2C_0_INST, &fw))
            uart_printf("Firmware Version : 0x%04X\n", fw);
        else
            uart_printf("Firmware Version : read failed\n");
    }

    else if (strcmp(sub, "dump") == 0) {
        uart_printf("=== BQ27Z746 Register Dump ===\n");
        uart_printf("Voltage          [0x08]: %4d mV\n",  BQ27Z746_ReadVoltage_mV(I2C_0_INST));
        uart_printf("Current          [0x0C]: %5d mA\n",  BQ27Z746_ReadCurrent_mA(I2C_0_INST));
        uart_printf("Avg Current      [0x14]: %5d mA\n",  BQ27Z746_ReadAvgCurrent_mA(I2C_0_INST));
        uart_printf("Avg Power        [0x22]: %5d mW\n",  BQ27Z746_ReadAvgPower_mW(I2C_0_INST));
        uart_printf("SOC              [0x2C]: %3d %%\n",   BQ27Z746_ReadSOC_pct(I2C_0_INST));
        uart_printf("Remaining Cap    [0x10]: %4d mAh\n", BQ27Z746_ReadRemainingCap_mAh(I2C_0_INST));
        uart_printf("Full Charge Cap  [0x12]: %4d mAh\n", BQ27Z746_ReadFullChargeCap_mAh(I2C_0_INST));
        uart_printf("State of Health  [0x2E]: %3d %%\n",   BQ27Z746_ReadStateOfHealth_pct(I2C_0_INST));
        uart_printf("Temperature      [0x06]: %3d C\n",   BQ27Z746_ReadTemperature_C(I2C_0_INST));
        uart_printf("Internal Temp    [0x28]: %3d C\n",   BQ27Z746_ReadInternalTemp_C(I2C_0_INST));

        uint16_t tte = BQ27Z746_ReadTimeToEmpty_min(I2C_0_INST);
        uint16_t ttf = BQ27Z746_ReadTimeToFull_min(I2C_0_INST);
        uart_printf("Time to Empty    [0x16]: "); print_time(tte); uart_printf("\n");
        uart_printf("Time to Full     [0x18]: "); print_time(ttf); uart_printf("\n");

        uart_printf("Cycle Count      [0x2A]: %d\n",      BQ27Z746_ReadCycleCount(I2C_0_INST));
        uart_printf("Battery Status   [0x0A]: 0x%04X\n",  BQ27Z746_ReadBatteryStatus(I2C_0_INST));
    }

    else if (strcmp(sub, "status") == 0) {
        uint16_t status = BQ27Z746_ReadBatteryStatus(I2C_0_INST);
        print_battery_status(status);
    }

    else if (strcmp(sub, "info") == 0) {
        uart_printf("=== BQ27Z746 Device Info ===\n");

        uint16_t dev_type = 0u;
        if (BQ27Z746_GetDeviceType(I2C_0_INST, &dev_type))
            uart_printf("Device Type      : 0x%04X %s\n", dev_type,
                        (dev_type == BQ27Z746_DEVICE_TYPE) ? "(OK)" : "(MISMATCH)");
        else
            uart_printf("Device Type      : read failed\n");

        uint16_t fw = 0u;
        if (BQ27Z746_GetFirmwareVersion(I2C_0_INST, &fw))
            uart_printf("Firmware Version : 0x%04X\n", fw);
        else
            uart_printf("Firmware Version : read failed\n");

        uint16_t chem = 0u;
        if (BQ27Z746_GetChemID(I2C_0_INST, &chem))
            uart_printf("Chem ID          : 0x%04X\n", chem);
        else
            uart_printf("Chem ID          : read failed\n");

        /* Single read — split into statusA / statusB locally */
        uint32_t op_status = 0u;
        if (BQ27Z746_GetOperationStatus(I2C_0_INST, &op_status)) {
            uint16_t statusA = (uint16_t)(op_status & 0xFFFFu);
            uint16_t statusB = (uint16_t)(op_status >> 16u);
            uart_printf("Operation Status : 0x%08X\n", (unsigned int)op_status);
            uart_printf("  Status A       : 0x%04X\n", statusA);
            uart_printf("  Status B       : 0x%04X\n", statusB);
        } else {
            uart_printf("Operation Status : read failed\n");
        }

        uint8_t tempRange = 0u;
        uint16_t chgStatus = 0u;
        if (BQ27Z746_GetChargingStatus(I2C_0_INST, &tempRange, &chgStatus)) {
            uart_printf("Temp Range       : 0x%02X\n", tempRange);
            uart_printf("Chg Status       : 0x%04X\n", chgStatus);
        } else {
            uart_printf("Charging Status  : read failed\n");
        }

        uint32_t safety_status = 0u;
        if (BQ27Z746_GetSafetyStatus(I2C_0_INST, &safety_status))
            uart_printf("Safety Status    : 0x%08X\n", (unsigned int)safety_status);
        else
            uart_printf("Safety Status    : read failed\n");

        uint8_t tempCfg = 0u;
        if (BQ27Z746_GetTempConfig(I2C_0_INST, &tempCfg)) {
            uart_printf("Temp Config      : 0x%02X\n", tempCfg);
            uart_printf("  TSInt (int)    : %s\n", (tempCfg & (1u << 0)) ? "ENABLED" : "disabled");
            uart_printf("  TS1   (ext)    : %s\n", (tempCfg & (1u << 1)) ? "ENABLED" : "disabled");
            uart_printf("  TS2   (GPO)    : %s\n", (tempCfg & (1u << 2)) ? "ENABLED" : "disabled");
        } else {
            uart_printf("Temp Config      : read failed\n");
        }
    }
    else if (strcmp(sub, "monitor") == 0) {
        gauge_monitor_active = true;
        uart_printf("Gauge monitor started — type any command to stop\n");
    }

    else if (strcmp(sub, "stop") == 0) {
        gauge_monitor_active = false;
        uart_printf("Gauge monitor stopped\n");
    }

    else {
        uart_printf("Unknown gauge sub-command. Type 'gauge' for help.\n");
    }
}




// ─────────────────────────────────────────────
// LED Control Command
// ─────────────────────────────────────────────
 
void cmd_leds(char *args) {
    char *tokens[2];
    int tokenCount = CLI_Tokenize(args, tokens, 2);
 
    if (tokenCount == 0) {
        uart_printf("LED Control CLI:\n"
                    "  led on <1|0>       - Enables and disables boost\n"
                    "  led init              - initialise LED control, both outputs zeroed\n"
                    "  led voltage <mV>      - set boost converter voltage (3490 - 11330 mV)\n"
                    "  led current <mA>      - set LED current (0 -2000 mA)\n"
                    "  led off               - safe shutdown, zeros current then voltage\n"
                    "  led flash <ms>          - start flashing, on-time 1-50 ms\n"
                    "  led flash stop          - stop flashing\n"
                    );
                    
        return;
    }
 
    char *sub = tokens[0];

    if (strcmp(sub, "on") == 0) {
    if (tokenCount < 2) {
        uart_printf("Usage: boost on <1|0>\n");
        return;
    }
        int state = atoi(tokens[1]);
        if (state) {
            enable_led_boost();
            uart_printf("Boost Enabled\n");
        } else {
            disable_led_boost();
            uart_printf("Boost disabled\n");
        }
    }
    // Initialise LED control state and zero both PWM outputs
    else if (strcmp(sub, "init") == 0) {
        LED_control_init();
        uart_printf("LED control initialised. Voltage: 0 mV, Current: 0 mA\n");
    }
 
    // Set boost converter output voltage
    else if (strcmp(sub, "voltage") == 0) {
        if (tokenCount < 2) {
            uart_printf("Usage: led voltage <mV>  (valid range: 3490 - 11330)\n");
            return;
        }
        uint16_t voltage = (uint16_t)atoi(tokens[1]);
        LED_set_voltage(voltage);
        uart_printf("LED voltage set to %d mV (applied: %d mV)\n", voltage, LED_get_voltage());
    }
 
    // Set LED output current
    else if (strcmp(sub, "current") == 0) {
        if (tokenCount < 2) {
            uart_printf("Usage: led current <mA>  (valid range: 0 - 2000)\n");
            return;
        }
        uint16_t current = (uint16_t)atoi(tokens[1]);
        LED_set_current(current);
        uart_printf("LED current set to %d mA (applied: %d mA)\n", current, LED_get_current());
    }
 
    // Safe shutdown — zero current first then voltage
    else if (strcmp(sub, "off") == 0) {
        LED_set_current(0);
        // LED_set_voltage(0);
        disable_led_boost();
        uart_printf("LED off. Current zeroed then voltage zeroed.\n");
    }
    else if (strcmp(sub, "flash") == 0) {
    if (tokenCount < 2) {
        uart_printf("Usage: led flash <ms>   (1 - 50 ms)\n"
                    "       led flash stop\n");
        return;
    }
    if (strcmp(tokens[1], "stop") == 0) {
        LED_flash_stop();
        uart_printf("Flash stopped\n");
    } else {
        uint16_t on_ms = (uint16_t)atoi(tokens[1]);
        LED_flash_start(on_ms);
        uart_printf("Flashing: on-time %d ms (ticks: %d)\n", on_ms, on_ms * 500);
    }
    }
    else {
        uart_printf("Unknown led sub-command. Type 'led' for help.\n");
    }
}
 

 // ─────────────────────────────────────────────
// PIR Monitor Command
// ─────────────────────────────────────────────
 
// ─────────────────────────────────────────────
// PIR Monitor Command
// ─────────────────────────────────────────────

static void pir_print_help(void) {
    uart_printf("Usage:\n");
    uart_printf("  pir init <bus> <addr> <type> <step> <threshold>\n");
    uart_printf("     bus       : 0 or 1\n");
    uart_printf("     addr      : 10-bit I2C address (e.g. 0x301)\n");
    uart_printf("     type      : A B C D DIRECT\n");
    uart_printf("     step      : 1 2 3\n");
    uart_printf("     threshold : 0-255 (actual = value * 8 ADC counts)\n");
    uart_printf("  pir status\n");
    uart_printf("  pir monitor\n");
    uart_printf("  pir reset\n");
}

static ZDP323B_FilterType parse_filter_type(const char *s) {
    if      (strcmp(s, "A")      == 0) return ZDP323B_FILTER_TYPE_A;
    else if (strcmp(s, "B")      == 0) return ZDP323B_FILTER_TYPE_B;
    else if (strcmp(s, "C")      == 0) return ZDP323B_FILTER_TYPE_C;
    else if (strcmp(s, "D")      == 0) return ZDP323B_FILTER_TYPE_D;
    else if (strcmp(s, "DIRECT") == 0) return ZDP323B_FILTER_TYPE_DIRECT;
    else                               return ZDP323B_FILTER_TYPE_B; // safe default
}

static ZDP323B_FilterStep parse_filter_step(int s) {
    if      (s == 1) return ZDP323B_FILTER_STEP_1;
    else if (s == 3) return ZDP323B_FILTER_STEP_3;
    else             return ZDP323B_FILTER_STEP_2; // default step 2
}

static const char* filter_type_str(ZDP323B_FilterType t) {
    switch (t) {
        case ZDP323B_FILTER_TYPE_A:      return "A";
        case ZDP323B_FILTER_TYPE_B:      return "B";
        case ZDP323B_FILTER_TYPE_C:      return "C";
        case ZDP323B_FILTER_TYPE_D:      return "D";
        case ZDP323B_FILTER_TYPE_DIRECT: return "DIRECT";
        default:                         return "?";
    }
}

static const char* filter_step_str(ZDP323B_FilterStep s) {
    switch (s) {
        case ZDP323B_FILTER_STEP_1: return "1";
        case ZDP323B_FILTER_STEP_2: return "2";
        case ZDP323B_FILTER_STEP_3: return "3";
        default:                    return "?";
    }
}

// ─────────────────────────────────────────────
// PIR Command
// ─────────────────────────────────────────────

void cmd_pir(char *args) {
    char *tokens[6];
    int tokenCount = CLI_Tokenize(args, tokens, 6);

    if (tokenCount == 0) {
        pir_print_help();
        return;
    }

    char *sub = tokens[0];

    // ── pir init <bus> <addr> <type> <step> <threshold> ──
    if (strcmp(sub, "init") == 0) {
        if (tokenCount < 6) {
            uart_printf("[PIR] init requires: bus addr type step threshold\n");
            pir_print_help();
            return;
        }

        int      busNum   = atoi(tokens[1]);
        uint16_t dev_addr = (uint16_t)strtol(tokens[2], NULL, 0);
        ZDP323B_FilterType ftype = parse_filter_type(tokens[3]);
        ZDP323B_FilterStep fstep = parse_filter_step(atoi(tokens[4]));
        uint8_t  threshold = (uint8_t)atoi(tokens[5]);

        I2C_Regs *targetBus;
        if      (busNum == 0) targetBus = I2C_0_INST;
        else if (busNum == 1) targetBus = I2C_1_INST;
        else {
            uart_printf("[PIR] Invalid bus. Use 0 or 1.\n");
            return;
        }

        uart_printf("[PIR] Initializing on bus %d addr 0x%03X\n", busNum, dev_addr);
        uart_printf("[PIR] Filter: Type %s  Step %s  Threshold: %d (%d ADC counts)\n",
                    filter_type_str(ftype),
                    filter_step_str(fstep),
                    threshold,
                    threshold * 8);

        I2C_Status st = ZDP323B_Init(targetBus, dev_addr, fstep, ftype, threshold);
        if (st != I2C_SUCCESS) {
            uart_printf("[PIR] Init failed with status %d\n", st);
        }
    }

    // ── pir status ────────────────────────────────
    else if (strcmp(sub, "status") == 0) {
        if (!gPIR.initialized) {
            uart_printf("[PIR] Not initialized. Run 'pir init' first.\n");
            return;
        }

        int16_t peak = 0;
        I2C_Status st = ZDP323B_ReadPeakHold(gPIR.i2c, gPIR.dev_addr, &peak);
        if (st != I2C_SUCCESS) {
            uart_printf("[PIR] Failed to read Peak Hold\n");
            return;
        }

        uart_printf("[PIR] Status:\n");
        uart_printf("  Addr       : 0x%03X\n", gPIR.dev_addr);
        uart_printf("  Filter     : Type %s  Step %s\n",
                    filter_type_str(gPIR.armed_cfg.filter_type),
                    filter_step_str(gPIR.armed_cfg.filter_step));
        uart_printf("  Threshold  : %d (%d ADC counts)\n",
                    gPIR.armed_cfg.threshold,
                    gPIR.armed_cfg.threshold * 8);
        uart_printf("  Peak Hold  : %d\n", peak);
        uart_printf("  Motion Flag: %s\n", gPIR.motion_detected ? "SET" : "clear");
        uart_printf("  Monitor    : %s\n", pir_monitor_active   ? "running" : "stopped");
    }

    // ── pir monitor ───────────────────────────────
    else if (strcmp(sub, "monitor") == 0) {
        if (!gPIR.initialized) {
            uart_printf("[PIR] Not initialized. Run 'pir init' first.\n");
            return;
        }

        pir_monitor_active = true;
        uart_printf("[PIR] Monitor started. Send any key to stop.\n");
        uart_printf("%-10s %-10s %-12s\n", "Peak Hold", "Motion",  "Threshold");
        uart_printf("%-10s %-10s %-12s\n", "---------", "------", "---------");
    }

    // ── pir reset ─────────────────────────────────
    else if (strcmp(sub, "reset") == 0) {
        if (!gPIR.initialized) {
            uart_printf("[PIR] Not initialized.\n");
            return;
        }

        // Stop monitor if running
        pir_monitor_active = false;

        // Write default values per datasheet section 9.4 / 13
        ZDP323B_Config reset_cfg = {
            .threshold   = 0x38,
            .trigger_en  = false,
            .filter_step = ZDP323B_FILTER_STEP_2,
            .filter_type = ZDP323B_FILTER_TYPE_B,
        };

        uint8_t config_bytes[7];
        ZDP323B_BuildConfigBytes(&reset_cfg, config_bytes);
        I2C_Status st = ZDP323B_WriteConfig(gPIR.i2c, gPIR.dev_addr, config_bytes);
        if (st != I2C_SUCCESS) {
            uart_printf("[PIR] Reset write failed\n");
            return;
        }

        gPIR.motion_detected = false;
        gPIR.initialized     = false;

        uart_printf("[PIR] Reset complete. Re-run 'pir init' to use.\n");
    }

    else {
        uart_printf("[PIR] Unknown sub-command '%s'\n", sub);
        pir_print_help();
    }
}
 
void cmd_i2cscan10(char *args) {
    char *tokens[1];
    int tokenCount = CLI_Tokenize(args, tokens, 1);

    I2C_Regs *targetBus;
    int busNum = (tokenCount > 0) ? atoi(tokens[0]) : 0;

    if (busNum == 0)      targetBus = I2C_0_INST;
    else if (busNum == 1) targetBus = I2C_1_INST;
    else {
        uart_printf("Invalid bus. Use 0 or 1.\n");
        return;
    }

    uart_printf("Scanning I2C Bus %d (10-bit)...\n", busNum);
    uint8_t foundCount = 0;

    for (uint16_t addr = 0x000; addr <= 0x3FF; addr++) {
        if (I2C_TryAddress10(targetBus, addr)) {
            uart_printf("  Found device at 0x%03X\n", addr);
            foundCount++;
        }
    }

    if (foundCount == 0) {
        uart_printf("No devices found.\n");
    } else {
        uart_printf("Scan complete. %d device(s) found.\n", foundCount);
    }
}

// ─────────────────────────────────────────────
// LTR-329ALS-01 CLI Command
// ─────────────────────────────────────────────

void cmd_ltr(char *args) {
    char *tokens[3];
    int tokenCount = CLI_Tokenize(args, tokens, 3);

    if (tokenCount == 0) {
        uart_printf("LTR-329ALS-01 CLI:\n"
                    "  ltr init <bus>      - Initialize on I2C bus 0 or 1\n"
                    "  ltr read            - One-shot CH0, CH1 and Lux read\n"
                    "  ltr gain <val>      - Set gain: 1, 2, 4, 8, 48, 96\n"
                    "  ltr monitor         - 200ms live telemetry\n"
                    "  ltr model_monitor   - 500ms live model predictions (EXPOSURE, GAIN, LUX)\n"
                    "  ltr predict <lux>   - Predict exposure and gain for manual lux value\n"
                    "  ltr stop            - Stop monitor / model monitor\n");
        return;
    }

    char *sub = tokens[0];

    if (strcmp(sub, "init") == 0) {
        int busNum = (tokenCount > 1) ? atoi(tokens[1]) : 0;
        I2C_Regs *bus = (busNum == 1) ? I2C_1_INST : I2C_0_INST;
        
        if (LTR329_Init(bus)) {
            uart_printf("LTR-329 initialized successfully on I2C%d\n", busNum);
        } else {
            uart_printf("ERROR: LTR-329 initialization failed\n");
        }
    }
    else if (strcmp(sub, "read") == 0) {
        uint16_t ch0, ch1;
        if (LTR329_ReadData(&ch0, &ch1)) {
            float lux = LTR329_CalculateLux(ch0, ch1);
            uart_printf("CH0: %u  CH1: %u  Lux: %.2f\n", ch0, ch1, lux);
        } else {
            uart_printf("ERROR: Failed to read data\n");
        }
    }
    else if (strcmp(sub, "gain") == 0) {
        if (tokenCount < 2) {
            uart_printf("Usage: ltr gain <1|2|4|8|48|96>\n");
            return;
        }
        int gainVal = atoi(tokens[1]);
        LTR329_Gain gain;
        switch(gainVal) {
            case 1:  gain = LTR329_GAIN_1X;  break;
            case 2:  gain = LTR329_GAIN_2X;  break;
            case 4:  gain = LTR329_GAIN_4X;  break;
            case 8:  gain = LTR329_GAIN_8X;  break;
            case 48: gain = LTR329_GAIN_48X; break;
            case 96: gain = LTR329_GAIN_96X; break;
            default: uart_printf("Invalid gain value\n"); return;
        }
        if (LTR329_SetGain(gain)) {
            uart_printf("Gain set to %dX\n", gainVal);
        } else {
            uart_printf("ERROR: Failed to set gain\n");
        }
    }
    else if (strcmp(sub, "monitor") == 0) {
        ltr_monitor_active = true;
        uart_printf("LTR monitor started — type any command to stop\n");
    }
    else if (strcmp(sub, "model_monitor") == 0) {
        ltr_model_monitor_active = true;
        uart_printf("EXPOSURE , GAIN, LUX\r\n");
    }
    else if (strcmp(sub, "predict") == 0) {
        if (tokenCount < 2) {
            uart_printf("Usage: ltr predict <lux>\n");
            return;
        }
        float lux = (float)atof(tokens[1]);

        /* Time the inference as well as printing it. Both trees are deep
         * if/else chains on a Cortex-M0+ with no FPU, and the whole point of
         * the seed is that it is ready before FSBL asks - so how long this
         * takes is a number worth having, not a curiosity.
         *
         * Uses the shared time base rather than reprogramming SysTick directly,
         * as an earlier version of this command did. That version left SysTick
         * disabled on the way out, so running `ltr predict` while the state
         * machine was inside a STM32 power-on silently killed every boot-latency
         * number for that wake. One owner for the peripheral, no exceptions. */
        bool ticks_borrowed = false;
        if (!Ticks_Running()) {
            Ticks_Start();          /* CLI use outside a power-on window */
            ticks_borrowed = true;
        }

        uint32_t t0 = Ticks_us();

        double input[1];
        input[0] = log1p((double)lux);
        double exp_pred = score_exposure_sep(input);
        double gain_pred = score_gain_sep(input);

        uint32_t elapsed_us = Ticks_us() - t0;

        if (ticks_borrowed) {
            Ticks_Stop();
        }

        uart_printf("%ld, %ld, %ld\r\n", (int32_t)exp_pred, (int32_t)gain_pred, (int32_t)lux);
        uart_printf("Inference time: ~%lu us\r\n", (unsigned long)elapsed_us);
    }
    else if (strcmp(sub, "stop") == 0) {
        ltr_monitor_active = false;
        ltr_model_monitor_active = false;
        uart_printf("LTR monitor stopped\n");
    }
    else {
        uart_printf("Unknown ltr sub-command\n");
    }
}

void cmd_lis(char *args) {
    char *tokens[3];
    int tokenCount = CLI_Tokenize(args, tokens, 3);
    extern volatile bool lis_monitor_active;

    if (tokenCount == 0) {
        uart_printf("LIS3DH CLI:\n"
                    "  lis init <bus>      - Initialize on I2C bus 0 or 1\n"
                    "  lis read            - One-shot X, Y, Z (mg) read\n"
                    "  lis range <2|4|8|16>- Set full-scale range\n"
                    "  lis monitor         - 200ms live telemetry\n"
                    "  lis stop            - Stop monitor\n");
        return;
    }

    char *sub = tokens[0];

    if (strcmp(sub, "init") == 0) {
        int busNum = (tokenCount > 1) ? atoi(tokens[1]) : 0;
        I2C_Regs *bus = (busNum == 1) ? I2C_1_INST : I2C_0_INST;
        
        // Default to address 0x18
        if (LIS3DH_Init(bus, LIS3DH_I2C_ADDR_0)) {
            uart_printf("LIS3DH initialized successfully on I2C%d (addr 0x%02X)\n", busNum, LIS3DH_I2C_ADDR_0);
        } else {
            uart_printf("ERROR: LIS3DH initialization failed\n");
        }
    }
    else if (strcmp(sub, "read") == 0) {
        float x, y, z;
        if (LIS3DH_ReadMg(&x, &y, &z)) {
            uart_printf("X: %8.2f  Y: %8.2f  Z: %8.2f mg\n", x, y, z);
        } else {
            uart_printf("ERROR: Failed to read data\n");
        }
    }
    else if (strcmp(sub, "range") == 0) {
        if (tokenCount < 2) {
            uart_printf("Usage: lis range <2|4|8|16>\n");
            return;
        }
        int rVal = atoi(tokens[1]);
        LIS3DH_Range range;
        switch(rVal) {
            case 2:  range = LIS3DH_RANGE_2G;  break;
            case 4:  range = LIS3DH_RANGE_4G;  break;
            case 8:  range = LIS3DH_RANGE_8G;  break;
            case 16: range = LIS3DH_RANGE_16G; break;
            default: uart_printf("Invalid range. Use 2, 4, 8, or 16.\n"); return;
        }
        if (LIS3DH_SetRange(range)) {
            uart_printf("Range set to ±%dg\n", rVal);
        } else {
            uart_printf("ERROR: Failed to set range\n");
        }
    }
    else if (strcmp(sub, "monitor") == 0) {
        lis_monitor_active = true;
        uart_printf("LIS monitor started — type any command to stop\n");
    }
    else if (strcmp(sub, "stop") == 0) {
        lis_monitor_active = false;
        uart_printf("LIS monitor stopped\n");
    }
    else {
        uart_printf("Unknown lis sub-command\n");
    }
}

void cmd_imx(char *args) {
    char *tokens[3];
    int tokenCount = CLI_Tokenize(args, tokens, 3);

    if (tokenCount < 1) {
        uart_printf("IMX335 Camera Control CLI:\n"
                    "  imx scan             - Scan I2C1 for the camera\n"
                    "  imx init             - Initialize the camera (standby mode)\n"
                    "  imx start            - Start camera streaming\n"
                    "  imx stop             - Stop camera streaming (standby)\n"
                    "  imx id               - Read camera sensor ID\n"
                    "  imx read <reg_hex>   - Read a 16-bit register (hex address)\n"
                    "  imx write <reg_hex> <val_hex> - Write a value to a 16-bit register\n"
                    "  imx gain <mdB>       - Set gain in mdB (0 to 72000, e.g. 20000 for 20dB)\n"
                    "  imx exposure <us>    - Set exposure in microseconds (0 to 33266)\n"
                    "  imx tpg <mode>       - Set test pattern (-1:off, 10:H-bars, 11:V-bars)\n");
        return;
    }

    char *sub = tokens[0];

    if (strcmp(sub, "scan") == 0) {
        uart_printf("Scanning for IMX335 on I2C1...\n");
        if (IMX335_Scan()) {
            uart_printf("IMX335 camera found at 0x%02X\n", gIMX335.dev_addr);
        } else {
            uart_printf("IMX335 camera not found (checked 0x1A and 0x36)\n");
        }
    }
    else if (strcmp(sub, "init") == 0) {
        uart_printf("Initializing IMX335 on I2C1...\n");
        if (IMX335_Scan() == false) {
            uart_printf("ERROR: Camera not detected on I2C1\n");
            return;
        }
        if (IMX335_Init(I2C_1_INST)) {
            uart_printf("IMX335 initialized successfully. Camera is in Standby.\n");
        } else {
            uart_printf("ERROR: IMX335 initialization failed\n");
        }
    }
    else if (strcmp(sub, "start") == 0) {
        uart_printf("Starting IMX335 streaming...\n");
        if (IMX335_Start()) {
            uart_printf("IMX335 streaming started successfully.\n");
        } else {
            uart_printf("ERROR: Failed to start IMX335 streaming\n");
        }
    }
    else if (strcmp(sub, "stop") == 0) {
        uart_printf("Stopping IMX335 streaming...\n");
        if (IMX335_Stop()) {
            uart_printf("IMX335 streaming stopped (sensor put to standby).\n");
        } else {
            uart_printf("ERROR: Failed to stop IMX335 streaming\n");
        }
    }
    else if (strcmp(sub, "id") == 0) {
        uint32_t id = 0;
        if (IMX335_ReadID(&id)) {
            uart_printf("IMX335 Sensor ID: 0x%02X (Expected: 0x%02X)\n", id, IMX335_CHIP_ID);
        } else {
            uart_printf("ERROR: Failed to read Sensor ID\n");
        }
    }
    else if (strcmp(sub, "read") == 0) {
        if (tokenCount < 2) {
            uart_printf("Usage: imx read <reg_hex>\n");
            return;
        }
        uint16_t reg = (uint16_t)strtol(tokens[1], NULL, 16);
        uint8_t val = 0;
        if (IMX335_ReadReg(reg, &val)) {
            uart_printf("Reg [0x%04X] = 0x%02X\n", reg, val);
        } else {
            uart_printf("ERROR: Failed to read register 0x%04X\n", reg);
        }
    }
    else if (strcmp(sub, "write") == 0) {
        if (tokenCount < 3) {
            uart_printf("Usage: imx write <reg_hex> <val_hex>\n");
            return;
        }
        uint16_t reg = (uint16_t)strtol(tokens[1], NULL, 16);
        uint8_t val = (uint8_t)strtol(tokens[2], NULL, 16);
        if (IMX335_WriteReg(reg, val)) {
            uart_printf("Reg [0x%04X] written with 0x%02X\n", reg, val);
        } else {
            uart_printf("ERROR: Failed to write register 0x%04X\n", reg);
        }
    }
    else if (strcmp(sub, "gain") == 0) {
        if (tokenCount < 2) {
            uart_printf("Usage: imx gain <mdB>\n");
            return;
        }
        uint32_t gain = (uint32_t)atoi(tokens[1]);
        if (IMX335_SetGain(gain)) {
            uart_printf("IMX335 gain set to %u mdB\n", gain);
        } else {
            uart_printf("ERROR: Failed to set gain to %u mdB\n", gain);
        }
    }
    else if (strcmp(sub, "exposure") == 0) {
        if (tokenCount < 2) {
            uart_printf("Usage: imx exposure <us>\n");
            return;
        }
        uint32_t exp = (uint32_t)atoi(tokens[1]);
        if (IMX335_SetExposure(exp)) {
            uart_printf("IMX335 exposure set to %u us\n", exp);
        } else {
            uart_printf("ERROR: Failed to set exposure to %u us\n", exp);
        }
    }
    else if (strcmp(sub, "tpg") == 0) {
        if (tokenCount < 2) {
            uart_printf("Usage: imx tpg <mode>\n");
            return;
        }
        int32_t mode = (int32_t)atoi(tokens[1]);
        if (IMX335_SetTestPattern(mode)) {
            if (mode >= 0) {
                uart_printf("IMX335 Test Pattern Generator enabled (mode %d)\n", mode);
            } else {
                uart_printf("IMX335 Test Pattern Generator disabled\n");
            }
        } else {
            uart_printf("ERROR: Failed to configure Test Pattern Generator\n");
        }
    }
    else {
        uart_printf("Unknown imx sub-command. Type 'imx' for help.\n");
    }
}
