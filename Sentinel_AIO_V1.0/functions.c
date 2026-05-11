#include "ti_msp_dl_config.h"
#include "HAL/uart.h"
#include <stdlib.h>
#include <string.h>
#include "HAL/i2c.h"
#include "ics/BQ25628/BQ25628_functions.h"
#include "HAL/spi_master.h"
#include "ics/BQ27Z7/BQ27Z7_functions.h"
#include "sm.h"
extern volatile bool gauge_monitor_active;
extern volatile bool bq_monitor_active; 
extern volatile bool hall_monitor_active;
extern volatile uint32_t monitor_rate;

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
}

void cmd_sm(char *args) {
    char *tokens[1];
    int tokenCount = CLI_Tokenize(args, tokens, 1);

    if (tokenCount == 0) {
        uart_printf("SM Control CLI\n"
                    "  sm status  - print current state\n"
                    "  sm start   - resume state machine\n"
                    "  sm stop    - pause state machine\n");
        return;
    }

    char *sub = tokens[0];

if (strcmp(sub, "status") == 0) {
    uart_printf("=== State Machine ===\n");
    uart_printf("  State         : %s -> %s\n",
        sm_context.previous == SM_STATE_INIT           ? "INIT"           :
        sm_context.previous == SM_STATE_CHARGING       ? "CHARGING"       :
        sm_context.previous == SM_STATE_POWER_STM      ? "POWER_STM"      :
        sm_context.previous == SM_STATE_IDLE           ? "IDLE"           :
        sm_context.previous == SM_STATE_CRITICAL_FAULT ? "CRITICAL_FAULT" : "UNKNOWN",
        SM_GetStateString());
    uart_printf("  Wake Reason   : %s\n", sm_context.wake_reason == SM_WAKE_SETUP ? "SETUP" : "NORMAL");
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