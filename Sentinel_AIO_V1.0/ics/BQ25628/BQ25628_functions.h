#ifndef BQ25628E_H
#define BQ25628E_H
#include <stdint.h>
#include <stdbool.h>
#include "spi_protocol.h"
/* I2C Address */
#define BQ25628E_I2C_ADDR 0x6Au

#define RT1_KOHM            5.23f
#define RT2_KOHM            30.0f
#define TS_ADC_STEP_PCT     0.000961f  // 0.0961% per LSB

typedef struct {
    float temp;    // Temperature in °C
    float resist;  // Resistance in kΩ
} TempResistPair;


typedef enum {
    E3103JT2A,
    NTCC_10K
} NTC;

/* Charging Profile Configuration */
#define BQ_INIT_VREG_MV          3600   
#define BQ_INIT_ICHG_MA          1000    
#define BQ_INIT_IINDPM_MA        2000   
#define BQ_INIT_VINDPM_MV        4500   
#define BQ_INIT_VSYSMIN_MV       3600  

#define BQ_INIT_IPRECHG_MA       100    
#define BQ_INIT_ITERM_MA         50  


/* -------------------------------------------------------------------------- */
/* 16-bit Configuration Registers                                             */
/* -------------------------------------------------------------------------- */
#define BQ25628E_REG_ICHG      0x02u /* Charge Current Limit */
#define BQ25628E_REG_VREG      0x04u /* Charge Voltage Limit */
#define BQ25628E_REG_IINDPM    0x06u /* Input Current Limit */
#define BQ25628E_REG_VINDPM    0x08u /* Input Voltage Limit () */
#define BQ25628E_REG_VSYSMIN   0x0Eu /* Minimal System Voltage () */

/* -------------------------------------------------------------------------- */
/* 8-bit Control / Precharge / Termination Registers                          */
/* -------------------------------------------------------------------------- */
#define BQ25628E_REG_IPRECHG   0x10u /* Precharge Control  */
#define BQ25628E_REG_TERM_CTRL 0x12u /* Termination Control () */

#define BQ25628E_REG_CHG_CTRL  0x14u /* Charge Control */
#define BQ25628E_REG_CHG_TMR   0x15u /* Charge Timer Control */
#define BQ25628E_REG_CTRL0     0x16u /* Charger Control 0 */
#define BQ25628E_REG_CTRL1     0x17u /* Charger Control 1 */
#define BQ25628E_REG_CTRL2     0x18u /* Charger Control 2 */
#define BQ25628E_REG_CTRL3     0x19u /* Charger Control 3 (Peak Discharge) */
#define BQ25628E_REG_NTC0      0x1Au /* NTC Control 0 (TS disable) */
#define BQ25628E_REG_NTC1      0x1Bu /* NTC Control 1 */
#define BQ25628E_REG_NTC2      0x1Cu /* NTC Control 2 */

/* -------------------------------------------------------------------------- */
/* Status / Flag / Mask / ADC / Part Info                       */
/* -------------------------------------------------------------------------- */
#define BQ25628E_REG_STAT0       0x1Du
#define BQ25628E_REG_STAT1       0x1Eu
#define BQ25628E_REG_FAULT0      0x1Fu
#define BQ25628E_REG_CHG_FLAG0   0x20u
#define BQ25628E_REG_CHG_FLAG1   0x21u
#define BQ25628E_REG_FAULT_FLAG0 0x22u
#define BQ25628E_REG_CHG_MASK0   0x23u
#define BQ25628E_REG_CHG_MASK1   0x24u
#define BQ25628E_REG_FAULT_MASK0 0x25u

#define BQ25628E_REG_ADC_CTRL  0x26u
#define BQ25628E_REG_ADC_IBUS  0x28u
#define BQ25628E_REG_ADC_IBAT  0x2Au
#define BQ25628E_REG_ADC_VBUS  0x2Cu
#define BQ25628E_REG_ADC_VPMID 0x2Eu
#define BQ25628E_REG_ADC_VBAT  0x30u
#define BQ25628E_REG_ADC_VSYS  0x32u
#define BQ25628E_REG_ADC_TS    0x34u
#define BQ25628E_REG_TDIE_ADC  0x36u
#define BQ25628E_REG_PART_INFO 0x38u

/* -------------------------------------------------------------------------- */
/* Bit Definitions                                           */
/* -------------------------------------------------------------------------- */
#define BQ25628E_CTRL0_EN_CHG   (1u << 5)
#define BQ25628E_CTRL0_EN_HIZ   (1u << 4)
#define BQ25628E_CTRL0_WD_RST   (1u << 2)
#define BQ25628E_CTRL1_REG_RST  (1u << 7)
#define BQ25628E_ADC_EN         (1u << 7)

/* : Precharge & Termination */
#define BQ25628E_IPRECHG_MASK   0x1Fu
#define BQ25628E_ITERM_MASK     0xFCu   /* bits 7:2 */

/* : Charger Control 3 (Peak Discharge) */
#define BQ25628E_CTRL3_IBAT_PK_MASK (3u << 6)
#define BQ25628E_CTRL3_IBAT_PK_6A   (2u << 6)   /* 10b = 6 A */
#define BQ25628E_CTRL3_IBAT_PK_12A  (3u << 6)   /* 11b = 12 A (default) */

/* : NTC Control 0 (TS pin) */
#define BQ25628E_NTC0_TS_IGNORE (1u << 7)

/* : Charger Control 0 (Watchdog bits [1:0]) */
#define BQ25628E_CTRL0_WATCHDOG_MASK  (3u << 0)
#define BQ25628E_CTRL0_WATCHDOG_DIS   (0u << 0) // 00b = Disable
/* Charge Status Bits (Bits 4:3) */
#define BQ25628E_CHG_STAT_MASK      0x18u   /* 0001 1000b */
#define BQ25628E_CHG_STAT_NOT_CHG   0x00u   /* 00b: Not Charging / Terminated */
#define BQ25628E_CHG_STAT_FAST_CHG  0x08u   /* 01b: Trickle/Pre/Fast Charge */
#define BQ25628E_CHG_STAT_TAPER     0x10u   /* 10b: Taper Charge (CV mode) */
#define BQ25628E_CHG_STAT_TOP_OFF   0x18u   /* 11b: Top-off Timer Active */

/* VBUS Status Bits (Bits 2:0) */
#define BQ25628E_VBUS_STAT_MASK     0x07u   /* 0000 0111b */
#define BQ25628E_VBUS_STAT_NONE     0x00u   /* 000b: Not powered from VBUS */
#define BQ25628E_VBUS_STAT_UNKNOWN  0x04u   /* 100b: Unknown Adapter */

/* FAULT_FLAG0 Bit Definitions (Reg 0x22) */
#define BQ25628E_VBUS_FAULT_FLAG (1u << 7)
#define BQ25628E_BAT_FAULT_FLAG  (1u << 6)
#define BQ25628E_SYS_FAULT_FLAG  (1u << 5)
#define BQ25628E_TSHUT_FLAG      (1u << 3)
#define BQ25628E_TS_FLAG         (1u << 0)


/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */
bool BQ25628E_Init_Default(void);
void BQ25628E_UpdateTelemetry(void);
void BQ25628E_PetWatchdog(void);

/* Getters (unchanged) */
uint16_t BQ25628E_Get_VBAT_mV(void);
uint16_t BQ25628E_Get_VSYS_mV(void);
uint16_t BQ25628E_Get_VBUS_mV(void);
int16_t  BQ25628E_Get_IBUS_mA(void);
int16_t  BQ25628E_Get_IBAT_mA(void);
int16_t  BQ25628E_Get_TDIE_C(void);
float BQ25628E_Get_TBAT_C(void);
uint8_t BQ25628E_GetFaultFlags(void);

/* Setters (old + all the  ones you asked for) */

void BQ25628E_Set_VREG_mV(uint16_t voltage_mV);
void BQ25628E_Set_ICHG_mA(uint16_t current_mA);
void BQ25628E_Set_VREG_mV(uint16_t voltage_mV);
void BQ25628E_Set_IINDPM_mA(uint16_t current_mA);
void BQ25628E_Set_VINDPM_mV(uint16_t voltage_mV);      
void BQ25628E_Set_VSYSMIN_mV(uint16_t voltage_mV);     
void BQ25628E_Set_Precharge_mA(uint16_t current_mA);  
void BQ25628E_Set_Termination_mA(uint16_t current_mA); 
void BQ25628E_Set_ChargerEnable(bool enable);
void BQ25628E_Set_HIZ(bool enable);
void BQ25628E_Set_TS_Ignore(bool ignore);           
void BQ25628E_Set_PeakDischarge_6A(void);  
void BQ25628E_Disable_Watchdog(void);
void BQ25628E_ADC_Control(bool enable);

/* -------------------------------------------------------------------------- */
/* Low-level access (required for CLI dump/monitor/read/write)                */
/* -------------------------------------------------------------------------- */
extern uint8_t  BQ25628E_ReadReg8(uint8_t reg);
extern void     BQ25628E_WriteReg8(uint8_t reg, uint8_t val);
extern uint16_t BQ25628E_ReadReg16(uint8_t reg);
extern void     BQ25628E_WriteReg16(uint8_t reg, uint16_t val);
void BQ25628E_HardwareInit(void);
void BQ25628E_ApplyProfile(const SM_ChargerConfig_t *cfg);

#endif /* BQ25628E_H */