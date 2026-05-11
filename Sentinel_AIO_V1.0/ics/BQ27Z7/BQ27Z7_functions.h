#ifndef BQ27Z746_FUNCTIONS_H
#define BQ27Z746_FUNCTIONS_H

#include <stdint.h>
#include <stdbool.h>
#include "HAL/i2c.h"
#include "gauge.h"

// ================================================================
// Device Identity
// ================================================================
#define BQ27Z746_DEVICE_TYPE    0x7746u

// ================================================================
// Standard Command Registers
// ================================================================
#define BQ27Z746_REG_MANUFACTURERACCESS     0x00u
#define BQ27Z746_REG_ATRATE                 0x02u
#define BQ27Z746_REG_TEMPERATURE            0x06u
#define BQ27Z746_REG_VOLTAGE                0x08u
#define BQ27Z746_REG_BATTERYSTATUS          0x0Au
#define BQ27Z746_REG_CURRENT                0x0Cu
#define BQ27Z746_REG_REMAININGCAPACITY      0x10u
#define BQ27Z746_REG_FULLCHARGECAPACITY     0x12u
#define BQ27Z746_REG_AVERAGECURRENT         0x14u
#define BQ27Z746_REG_AVERAGETIMETOEMPTY     0x16u
#define BQ27Z746_REG_AVERAGETIMETOFULL      0x18u
#define BQ27Z746_REG_AVERAGEPOWER           0x22u
#define BQ27Z746_REG_INTERNALTEMPERATURE    0x28u
#define BQ27Z746_REG_CYCLECOUNT             0x2Au
#define BQ27Z746_REG_RELATIVESTATEOFCHARGE  0x2Cu
#define BQ27Z746_REG_STATEOFHEALTH          0x2Eu
#define BQ27Z746_REG_ALTMANUFACTURERACCESS  0x3Eu

// ================================================================
// MAC Command Codes
// ================================================================
#define BQ27Z746_MAC_DEVICETYPE         0x0001u
#define BQ27Z746_MAC_FIRMWAREVERSION    0x0002u
#define BQ27Z746_MAC_HARDWAREVERSION    0x0003u
#define BQ27Z746_MAC_CHEMID             0x0006u
#define BQ27Z746_MAC_RESET              0x0012u
#define BQ27Z746_MAC_OPERATIONSTATUS    0x0054u
#define BQ27Z746_MAC_CHARGINGSTATUS     0x0055u
#define BQ27Z746_MAC_GAUGINGSTATUS      0x0056u
#define BQ27Z746_MAC_SHUTDOWN           0x0010u // Command to enter shutdown
#define BQ27Z746_MAC_FET_ENABLE         0x0022u // Enable/Disable FET control
#define BQ27Z746_MAC_SEAL               0x0030u // Seal the gauge for production
#define BQ27Z746_MAC_SAFETYALERT        0x0050u // Real-time safety alerts
#define BQ27Z746_MAC_SAFETYSTATUS       0x0051u // Latched safety failures (OC, OV, UT, etc)
#define BQ27Z746_MAC_DASTATUS1          0x0071u // Cell voltages and power data

// ================================================================
// BatteryStatus bits (0x0A)
// ================================================================
#define BQ27Z746_STATUS_FD      (1u << 4)   // Fully Discharged
#define BQ27Z746_STATUS_FC      (1u << 5)   // Fully Charged
#define BQ27Z746_STATUS_DSG     (1u << 6)   // Discharging
#define BQ27Z746_STATUS_INIT    (1u << 7)
#define BQ27Z746_STATUS_RCA     (1u << 9)   // Remaining Capacity Alarm
#define BQ27Z746_STATUS_TDA     (1u << 11)  // Terminate Discharge Alarm
#define BQ27Z746_STATUS_OTA     (1u << 12)  // Over Temperature Alarm
#define BQ27Z746_STATUS_OCA     (1u << 14)  // Over Charge Alarm

// ================================================================
// OperationStatus bits (0x0054)
// ================================================================
// Temp Range Masks (Byte 0)
#define BQ27Z746_TEMP_OT  (1u << 6) // Overtemperature
#define BQ27Z746_TEMP_HT  (1u << 5) // High temperature
#define BQ27Z746_TEMP_STH (1u << 4) // Standard high
#define BQ27Z746_TEMP_RT  (1u << 3) // Recommended
#define BQ27Z746_TEMP_STL (1u << 2) // Standard low
#define BQ27Z746_TEMP_LT  (1u << 1) // Low temperature
#define BQ27Z746_TEMP_UT  (1u << 0) // Undertemperature

// Charging Status Masks (16-bit word)
#define BQ27Z746_CHG_VCT  (1u << 7) // Termination
#define BQ27Z746_CHG_SU   (1u << 5) // Suspend
#define BQ27Z746_CHG_IN   (1u << 4) // Inhibit
#define BQ27Z746_CHG_HV   (1u << 3) // High voltage region
#define BQ27Z746_CHG_MV   (1u << 2) // Mid voltage region
#define BQ27Z746_CHG_LV   (1u << 1) // Low voltage region
#define BQ27Z746_CHG_PV   (1u << 0) // Precharge region


// ================================================================
// SafetyStatus Bits (MAC 0x0051)
// ================================================================
// These are 32-bit latched safety failure flags

// Primary Safety Flags (Used in your SM_DecodeSafetyStatus function)
#define BQ27Z746_SAFETY_CUV   (1UL << 0)  // CUV: Cell Undervoltage
#define BQ27Z746_SAFETY_OVP   (1UL << 1)  // COV: Cell Overvoltage
#define BQ27Z746_SAFETY_OCC   (1UL << 2)  // OCC: Overcurrent During Charge
#define BQ27Z746_SAFETY_OCD   (1UL << 4)  // OCD: Overcurrent During Discharge
#define BQ27Z746_SAFETY_SCD   (1UL << 10) // HSCD: Hardware Short-Circuit Discharge

// Additional Temperature & Timeout Flags
#define BQ27Z746_SAFETY_HOCD  (1UL << 6)  // Overload during discharge
#define BQ27Z746_SAFETY_HOCC  (1UL << 8)  // Short-circuit during charge
#define BQ27Z746_SAFETY_OTC   (1UL << 12) // Over-temperature during charge
#define BQ27Z746_SAFETY_OTD   (1UL << 13) // Over-temperature during discharge
#define BQ27Z746_SAFETY_OTF   (1UL << 16) // Over-temperature FET
#define BQ27Z746_SAFETY_PTO   (1UL << 18) // Precharge Timeout
#define BQ27Z746_SAFETY_CTO   (1UL << 20) // Charge Timeout
#define BQ27Z746_SAFETY_UTC   (1UL << 26) // Under-temperature during charge
#define BQ27Z746_SAFETY_UTD   (1UL << 27) // Under-temperature during discharge

#define BQ27Z746_SAFETY_HCOV  (1UL << 30) // Hardware Cell Overvoltage
#define BQ27Z746_SAFETY_HCUV  (1UL << 31) // Hardware Cell Undervoltage



// ================================================================
// MAC frame constants
// ================================================================
#define BQ27Z746_MAC_DATA_LEN       32u
#define BQ27Z746_MAC_OVERHEAD       4u
#define BQ27Z746_MAC_FRAME_LEN      (BQ27Z746_MAC_DATA_LEN + BQ27Z746_MAC_OVERHEAD)

// ================================================================
// Telemetry cache
// ================================================================
typedef struct {
    uint16_t voltage_mV;
    int16_t  current_mA;
    int16_t  avgCurrent_mA;
    uint8_t  soc_pct;
    uint16_t remainingCap_mAh;
    uint16_t fullChargeCap_mAh;
    uint8_t  stateOfHealth_pct;
    int16_t  temperature_C;
    int16_t  internalTemp_C;
    uint16_t timeToEmpty_min;
    uint16_t timeToFull_min;
    uint16_t cycleCount;
    int16_t  avgPower_mW;
    uint16_t batteryStatus;
} BQ27Z746_Telemetry_t;

// ================================================================
// Public API
// ================================================================
bool BQ27Z746_Init(I2C_Regs *i2c);

// MAC layer
bool BQ27Z746_MAC_Read(I2C_Regs *i2c, uint16_t cmd, uint8_t *pData, uint8_t *pLen);
bool BQ27Z746_MAC_Send(I2C_Regs *i2c, uint16_t cmd);
bool BQ27Z746_MAC_Write(I2C_Regs *i2c, uint16_t cmd, uint8_t *pData, uint8_t data_len);

// Diagnostic MAC reads
bool BQ27Z746_GetDeviceType(I2C_Regs *i2c, uint16_t *pType);
bool BQ27Z746_GetFirmwareVersion(I2C_Regs *i2c, uint16_t *pVersion);
bool BQ27Z746_GetChemID(I2C_Regs *i2c, uint16_t *pChemID);
bool BQ27Z746_GetOperationStatus(I2C_Regs *i2c, uint32_t *pStatus);
bool BQ27Z746_GetChargingStatus(I2C_Regs *i2c, uint8_t *pTempRange, uint16_t *pChgStatus);
bool BQ27Z746_GetGaugingStatus(I2C_Regs *i2c, uint32_t *pStatus);
bool BQ27Z746_GetSafetyStatus(I2C_Regs *i2c, uint32_t *pStatus);

// Live reads
uint16_t BQ27Z746_ReadVoltage_mV(I2C_Regs *i2c);
int16_t  BQ27Z746_ReadCurrent_mA(I2C_Regs *i2c);
int16_t  BQ27Z746_ReadAvgCurrent_mA(I2C_Regs *i2c);
uint8_t  BQ27Z746_ReadSOC_pct(I2C_Regs *i2c);
uint16_t BQ27Z746_ReadRemainingCap_mAh(I2C_Regs *i2c);
uint16_t BQ27Z746_ReadFullChargeCap_mAh(I2C_Regs *i2c);
uint8_t  BQ27Z746_ReadStateOfHealth_pct(I2C_Regs *i2c);
int16_t  BQ27Z746_ReadTemperature_C(I2C_Regs *i2c);
uint16_t BQ27Z746_ReadCycleCount(I2C_Regs *i2c);
uint16_t BQ27Z746_ReadBatteryStatus(I2C_Regs *i2c);
int16_t  BQ27Z746_ReadInternalTemp_C(I2C_Regs *i2c);
uint16_t BQ27Z746_ReadTimeToEmpty_min(I2C_Regs *i2c);
uint16_t BQ27Z746_ReadTimeToFull_min(I2C_Regs *i2c);
int16_t  BQ27Z746_ReadAvgPower_mW(I2C_Regs *i2c);


// Cache
void     BQ27Z746_UpdateTelemetry(I2C_Regs *i2c);
uint16_t BQ27Z746_Get_Voltage_mV(void);
int16_t  BQ27Z746_Get_Current_mA(void);
int16_t  BQ27Z746_Get_AvgCurrent_mA(void);
uint8_t  BQ27Z746_Get_SOC_pct(void);
uint16_t BQ27Z746_Get_RemainingCap_mAh(void);
uint16_t BQ27Z746_Get_FullChargeCap_mAh(void);
uint8_t  BQ27Z746_Get_StateOfHealth_pct(void);
int16_t  BQ27Z746_Get_Temperature_C(void);
uint16_t BQ27Z746_Get_CycleCount(void);
uint16_t BQ27Z746_Get_BatteryStatus(void);

// Status helpers
bool BQ27Z746_IsFullyCharged(void);
bool BQ27Z746_IsFullyDischarged(void);
bool BQ27Z746_IsDischarging(void);

// Golden Image support (highly recommended)
bool BQ27Z746_LoadGoldenImage(I2C_Regs *i2c, const char *fs_string);

uint16_t BQ27Z746_Get_TimeToEmpty_min(void);
uint16_t BQ27Z746_Get_TimeToFull_min(void);
int16_t  BQ27Z746_Get_AvgPower_mW(void);
int16_t  BQ27Z746_Get_InternalTemp_C(void);



bool BQ27Z746_SetUTFET_Direct(I2C_Regs *i2c, bool enable);
bool BQ27Z746_GetFETOptions(I2C_Regs *i2c, uint16_t *pOutValue);
bool BQ27Z746_GetTempConfig(I2C_Regs *i2c, uint8_t *pTempEnable);
bool BQ27Z746_UseInternalTempOnly(I2C_Regs *i2c);

/*
 * BQ27Z746 Security / Unseal Layer
 *
 * The BQ27Z746 has three security levels (TRM §10.3):
 *
 *   SEALED      SEC1=1, SEC0=1  — DF and extended MAC inaccessible
 *   UNSEALED    SEC1=1, SEC0=0  — full DF read/write access
 *   FULL ACCESS SEC1=0, SEC0=1  — same as UNSEALED + boot-ROM access
 *
 * SEC1:SEC0 live in OperationStatusA (MAC 0x0054), bits [9:8].
 *
 * Unsealing is a two-step MAC write:
 *   1. Write KEY_WORD1 to AltManufacturerAccess (0x3E)
 *   2. Write KEY_WORD2 to AltManufacturerAccess (0x3E)
 *   No other write may occur between the two steps.
 *
 * The factory default keys are device-specific and not published in the
 * TRM. TI ships the device in FULL ACCESS mode. If your device has been
 * sealed with the factory defaults, use the values below. If your
 * production flow has changed the keys via MAC 0x0035 SecurityKeys(),
 * replace these constants with your own.
 *
 * To re-seal after DF writes, call BQ27Z746_Seal().
 */

// ================================================================
// Factory-default unseal keys (TI BQ27Z746 — change if customised)
// Each word is sent little-endian as a 2-byte MAC write.
// ================================================================
#define BQ27Z746_UNSEAL_KEY1        0x0414u   /* first word  */
#define BQ27Z746_UNSEAL_KEY2        0x3672u   /* second word */

// Full-Access keys (only needed if you need boot-ROM access)
#define BQ27Z746_FULLACCESS_KEY1    0xFFFFu
#define BQ27Z746_FULLACCESS_KEY2    0xFFFFu

// ================================================================
// Security-level constants (SEC1:SEC0 encoding)
// ================================================================
#define BQ27Z746_SEC_RESERVED       0x00u   /* 0,0 — do not use         */
#define BQ27Z746_SEC_FULL_ACCESS    0x01u   /* 0,1 — shipped default    */
#define BQ27Z746_SEC_UNSEALED       0x02u   /* 1,0                      */
#define BQ27Z746_SEC_SEALED         0x03u   /* 1,1                      */

// ================================================================
// Public API
// ================================================================

/*
 * BQ27Z746_GetSecurityMode
 * Reads OperationStatusA (MAC 0x0054) and extracts SEC1:SEC0.
 * Returns one of BQ27Z746_SEC_* constants.
 * Returns 0xFF on I2C error.
 */
uint8_t BQ27Z746_GetSecurityMode(I2C_Regs *i2c);

/*
 * BQ27Z746_IsSealed
 * Convenience wrapper — returns true when SEC1=1, SEC0=1.
 */
bool BQ27Z746_IsSealed(I2C_Regs *i2c);

/*
 * BQ27Z746_Unseal
 * Transitions SEALED → UNSEALED using the two-word unseal key.
 * Safe to call when already UNSEALED or FULL ACCESS (no-op).
 * Returns true on success, false if the device remains sealed.
 *
 * key1 / key2: the two 16-bit key words (use the #defines above
 *              unless your production flow changed them).
 */
bool BQ27Z746_Unseal(I2C_Regs *i2c, uint16_t key1, uint16_t key2);

/*
 * BQ27Z746_Seal
 * Sends MAC 0x0030 SealDevice to return to SEALED mode.
 * Call this after finishing DF writes in production.
 */
bool BQ27Z746_Seal(I2C_Regs *i2c);

/*
 * BQ27Z746_EnsureUnsealed
 * Checks current security level; unseals only if necessary.
 * Returns true if the device is UNSEALED or FULL ACCESS on exit.
 * Preferred helper to call before any DF write operation.
 */
bool BQ27Z746_EnsureUnsealed(I2C_Regs *i2c, uint16_t key1, uint16_t key2);


#endif