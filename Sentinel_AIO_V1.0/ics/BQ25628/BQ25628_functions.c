#include "BQ25628_functions.h"
#include "HAL/i2c.h"
#include <stdbool.h>
#include <math.h>


/* Internal Telemetry Cache */
static uint16_t g_vbat_mV = 0;
static uint16_t g_vsys_mV = 0;
static uint16_t g_vbus_mV = 0;
static int16_t  g_ibus_mA = 0;
static int16_t  g_ibat_mA = 0;
static int16_t g_tdie_C = 0;
static float g_tbat_C = 0.0f;

/* -------------------------------------------------------------------------- */
/* Low-Level I2C Helpers (unchanged)                                          */
/* -------------------------------------------------------------------------- */
uint8_t BQ25628E_ReadReg8(uint8_t reg) {
    uint8_t val = 0;
    I2C_ReadDevice(I2C_0_INST, BQ25628E_I2C_ADDR, reg, &val, 1);
    return val;
}

void BQ25628E_WriteReg8(uint8_t reg, uint8_t val) {
    I2C_WriteDevice(I2C_0_INST, BQ25628E_I2C_ADDR, reg, &val, 1);
}

uint16_t BQ25628E_ReadReg16(uint8_t reg) {
    uint8_t data[2] = {0, 0};
    I2C_ReadDevice(I2C_0_INST, BQ25628E_I2C_ADDR, reg, data, 2);
    return (uint16_t)((data[1] << 8) | data[0]);
}

void BQ25628E_WriteReg16(uint8_t reg, uint16_t val) {
    uint8_t data[2];
    data[0] = (uint8_t)(val & 0xFF);
    data[1] = (uint8_t)((val >> 8) & 0xFF);
    I2C_WriteDevice(I2C_0_INST, BQ25628E_I2C_ADDR, reg, data, 2);
}

void BQ25628E_UpdateBits8(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t old = BQ25628E_ReadReg8(reg);
    uint8_t v = (old & ~mask) | (value & mask);
    if (v != old) BQ25628E_WriteReg8(reg, v);
}

// the lookup table from your data
const TempResistPair thermistor_table_E3103JT2A[] = {
    {-40, 332.094}, {-35, 239.900}, {-30, 175.200}, {-25, 129.287}, 
    {-20, 96.358}, {-15, 72.500}, {-10, 55.046}, {-5, 42.157},
    {0, 32.554}, {5, 25.339}, {10, 19.872}, {15, 15.698},
    {20, 12.488}, {25, 10.000}, {30, 8.059}, {35, 6.535},
    {40, 5.330}, {45, 4.372}, {50, 3.605}, {55, 2.989},
    {60, 2.490}, {65, 2.084}, {70, 1.753}, {75, 1.481},
    {80, 1.256}, {85, 1.070}, {90,0.915}, {95, 0.786},
    {100, 0.677}, {105, 0.585}, {110, 0.508}, {115, 0.442},
    {120, 0.386}, {125, 0.338}, {130, 0.297}, {135, 0.262},
    {140,  0.231}, {145, 0.205}, {150, 0.182}
};

// Updated NTCC_10K (103AT-type) thermistor lookup table
// Source: your corrected data (156 entries, -30 °C to +125 °C)
const TempResistPair thermistor_table_ntcc_10k[] = {
    {-30, 193.5f},   {-29, 181.461f}, {-28, 170.268f}, {-27, 159.85f},
    {-26, 150.146f}, {-25, 141.1f},   {-24, 132.659f}, {-23, 124.779f},
    {-22, 117.418f}, {-21, 110.537f}, {-20, 104.101f}, {-19, 98.079f},
    {-18, 92.44f},   {-17, 87.159f},  {-16, 82.21f},   {-15, 77.571f},
    {-14, 73.219f},  {-13, 69.137f},  {-12, 65.305f},  {-11, 61.707f},
    {-10, 58.328f},  {-9,  55.153f},  {-8,  52.169f},  {-7,  49.363f},
    {-6,  46.724f},  {-5,  44.241f},  {-4,  41.904f},  {-3,  39.704f},
    {-2,  37.633f},  {-1,  35.681f},  {0,   33.8f},    {1,   32.108f},
    {2,   30.474f},  {3,   28.932f},  {4,   27.477f},  {5,   26.104f},
    {6,   24.807f},  {7,   23.583f},  {8,   22.426f},  {9,   21.333f},
    {10,  20.3f},    {11,  19.322f},  {12,  18.398f},  {13,  17.523f},
    {14,  16.695f},  {15,  15.912f},  {16,  15.169f},  {17,  14.466f},
    {18,  13.799f},  {19,  13.167f},  {20,  12.568f},  {21,  11.999f},
    {22,  11.46f},   {23,  10.948f},  {24,  10.461f},  {25,  10.0f},
    {26,  9.561f},   {27,  9.144f},   {28,  8.747f},   {29,  8.37f},
    {30,  8.011f},   {31,  7.67f},    {32,  7.345f},   {33,  7.036f},
    {34,  6.741f},   {35,  6.461f},   {36,  6.193f},   {37,  5.938f},
    {38,  5.695f},   {39,  5.464f},   {40,  5.242f},   {41,  5.031f},
    {42,  4.83f},    {43,  4.638f},   {44,  4.454f},   {45,  4.279f},
    {46,  4.111f},   {47,  3.951f},   {48,  3.797f},   {49,  3.651f},
    {50,  3.511f},   {51,  3.377f},   {52,  3.248f},   {53,  3.126f},
    {54,  3.008f},   {55,  2.896f},   {56,  2.788f},   {57,  2.684f},
    {58,  2.585f},   {59,  2.49f},    {60,  2.4f},     {61,  2.312f},
    {62,  2.229f},   {63,  2.148f},   {64,  2.071f},   {65,  1.997f},
    {66,  1.927f},   {67,  1.858f},   {68,  1.793f},   {69,  1.73f},
    {70,  1.67f},    {71,  1.612f},   {72,  1.556f},   {73,  1.503f},
    {74,  1.451f},   {75,  1.402f},   {76,  1.354f},   {77,  1.309f},
    {78,  1.265f},   {79,  1.222f},   {80,  1.181f},   {81,  1.142f},
    {82,  1.104f},   {83,  1.068f},   {84,  1.033f},   {85,  1.0f},
    {86,  0.967f},   {87,  0.936f},   {88,  0.906f},   {89,  0.877f},
    {90,  0.849f},   {91,  0.822f},   {92,  0.796f},   {93,  0.771f},
    {94,  0.747f},   {95,  0.723f},   {96,  0.701f},   {97,  0.679f},
    {98,  0.658f},   {99,  0.638f},   {100, 0.6f},     {101, 0.6f},
    {102, 0.582f},   {103, 0.564f},   {104, 0.548f},   {105, 0.531f},
    {106, 0.516f},   {107, 0.5f},     {108, 0.486f},   {109, 0.472f},
    {110, 0.458f},   {111, 0.445f},   {112, 0.432f},   {113, 0.419f},
    {114, 0.407f},   {115, 0.396f},   {116, 0.385f},   {117, 0.374f},
    {118, 0.364f},   {119, 0.353f},   {120, 0.344f},   {121, 0.334f},
    {122, 0.325f},   {123, 0.316f},   {124, 0.308f},   {125, 0.3f}
};

float _calculateTempFromRt(float Rt, NTC ntc) {
    const TempResistPair* thermistor_table = NULL;
    uint16_t table_size = 0;

    if (ntc == E3103JT2A) {
        thermistor_table = thermistor_table_E3103JT2A;
        table_size = sizeof(thermistor_table_E3103JT2A) / sizeof(TempResistPair);
    }else if (ntc == NTCC_10K) {
        thermistor_table = thermistor_table_ntcc_10k;
        table_size = sizeof(thermistor_table_ntcc_10k) / sizeof(TempResistPair);
    } 
    if (thermistor_table == NULL) return -300.0f;

    // Handle out-of-range
    if (Rt >= thermistor_table[0].resist) return thermistor_table[0].temp;
    if (Rt <= thermistor_table[table_size-1].resist) return thermistor_table[table_size-1].temp;

    // Linear Interpolation
    int j;
    for (j = 0; j < table_size - 1; j++) {
        if (Rt <= thermistor_table[j].resist && Rt >= thermistor_table[j+1].resist) {
            break;
        }
    }

    float temp_diff = thermistor_table[j+1].temp - thermistor_table[j].temp;
    float res_diff = thermistor_table[j+1].resist - thermistor_table[j].resist;
    
    return thermistor_table[j].temp + (Rt - thermistor_table[j].resist) * (temp_diff / res_diff);
}

/* -------------------------------------------------------------------------- */
/* Public API Implementation                                       */
/* -------------------------------------------------------------------------- */
bool BQ25628E_Init_Default(void) {
    BQ25628E_Set_VREG_mV(BQ_INIT_VREG_MV);
    BQ25628E_Set_ICHG_mA(BQ_INIT_ICHG_MA);
    BQ25628E_Set_IINDPM_mA(BQ_INIT_IINDPM_MA);
    BQ25628E_Set_VINDPM_mV(BQ_INIT_VINDPM_MV);
    BQ25628E_Set_VSYSMIN_mV(BQ_INIT_VSYSMIN_MV);
    BQ25628E_Set_Precharge_mA(BQ_INIT_IPRECHG_MA);   
    BQ25628E_Set_Termination_mA(BQ_INIT_ITERM_MA); 
    // BQ25628E_Set_TS_Ignore(true); 
    BQ25628E_Set_PeakDischarge_6A(); 

    BQ25628E_WriteReg8(BQ25628E_REG_ADC_CTRL, BQ25628E_ADC_EN);
    BQ25628E_Disable_Watchdog();
    return true;
}

void BQ25628E_HardwareInit(void)
{
    // BQ25628E_WriteReg8(BQ25628E_REG_ADC_CTRL, BQ25628E_ADC_EN);
    BQ25628E_Disable_Watchdog();
    // BQ25628E_Set_TS_Ignore(true);
    BQ25628E_Set_PeakDischarge_6A();
}

void BQ25628E_ApplyProfile(const SM_ChargerConfig_t *cfg)
{
    BQ25628E_Set_VREG_mV(cfg->vreg_mV);
    BQ25628E_Set_ICHG_mA(cfg->ichg_mA);
    BQ25628E_Set_IINDPM_mA(cfg->iindpm_mA);
    BQ25628E_Set_VINDPM_mV(cfg->vindpm_mV);
    BQ25628E_Set_VSYSMIN_mV(cfg->vsysmin_mV);
    BQ25628E_Set_Precharge_mA(cfg->iprechg_mA);
    BQ25628E_Set_Termination_mA(cfg->iterm_mA);
}

void BQ25628E_UpdateTelemetry(void) {
    uint16_t raw;

    raw = BQ25628E_ReadReg16(BQ25628E_REG_ADC_VBAT);
    uint16_t vbat_code = (raw >> 1) & 0x0FFF;
    g_vbat_mV = (uint16_t) roundf(vbat_code * 1.99f); 

    raw = BQ25628E_ReadReg16(BQ25628E_REG_ADC_VSYS);
    uint16_t vsys_code = (raw >> 1) & 0x0FFF;
    g_vsys_mV = (uint16_t) roundf(vsys_code * 1.99f);

    raw = BQ25628E_ReadReg16(BQ25628E_REG_ADC_VBUS);
    uint16_t vbus_code = (raw >> 2) & 0x1FFF;
    g_vbus_mV = (uint16_t) roundf(vbus_code * 3.97f);

    raw = BQ25628E_ReadReg16(BQ25628E_REG_ADC_IBUS);
    int16_t ibus_code = (int16_t) (raw >> 1);
    g_ibus_mA = ibus_code * 2;

    raw = BQ25628E_ReadReg16(BQ25628E_REG_ADC_IBAT);
    if (raw == 0x8000) {
        g_ibat_mA = 0;
    } else {
        int16_t ibat_code = (int16_t) (raw >> 2);
        g_ibat_mA = ibat_code * 4;
    }
    raw = BQ25628E_ReadReg16(BQ25628E_REG_TDIE_ADC);
    int16_t tdie_code = (int16_t)(raw & 0x0FFFu);
    if (tdie_code > 0x7FF) tdie_code -= 0x1000;
    g_tdie_C = (int16_t)roundf(tdie_code * 0.5f);

    raw = BQ25628E_ReadReg16(BQ25628E_REG_ADC_TS);
    uint16_t ts_code = raw & 0x0FFF;     
    float ts_pct = ts_code * TS_ADC_STEP_PCT;
    if (ts_pct > 0.84f) {
        g_tbat_C = -100.0f; // NTC Open
    } else {
        float r_bottom = RT1_KOHM * (ts_pct / (1.0f - ts_pct));
        float r_ntc = (r_bottom * RT2_KOHM) / (RT2_KOHM - r_bottom);
        g_tbat_C = _calculateTempFromRt(r_ntc, NTCC_10K);
    }
}
void BQ25628E_PetWatchdog(void) {
    BQ25628E_UpdateBits8(BQ25628E_REG_CTRL0, BQ25628E_CTRL0_WD_RST, BQ25628E_CTRL0_WD_RST);
}

/* --- Setters --- */

void BQ25628E_Set_VREG_mV(uint16_t voltage_mV) {
    if (voltage_mV < 3500) voltage_mV = 3500;
    if (voltage_mV > 4800) voltage_mV = 4800;
    uint16_t code = voltage_mV / 10u;     
    uint16_t reg  = code << 3u;
    BQ25628E_WriteReg16(BQ25628E_REG_VREG, reg);
}

void BQ25628E_Set_ICHG_mA(uint16_t current_mA) {
    if (current_mA < 40)   current_mA = 40;
    if (current_mA > 2000) current_mA = 2000;
    uint16_t code = current_mA / 40u;    
    uint16_t reg  = code << 5u;
    BQ25628E_WriteReg16(BQ25628E_REG_ICHG, reg);
}


void BQ25628E_Set_IINDPM_mA(uint16_t current_mA) {
    if (current_mA < 100)  current_mA = 100;
    if (current_mA > 3200) current_mA = 3200;
    uint16_t code = current_mA / 20u;   
    uint16_t reg  = code << 4u;
    BQ25628E_WriteReg16(BQ25628E_REG_IINDPM, reg);
}

void BQ25628E_Set_VINDPM_mV(uint16_t voltage_mV) {
    if (voltage_mV < 3900) voltage_mV = 3900;   
    if (voltage_mV > 16800) voltage_mV = 16800;
    uint16_t code = voltage_mV / 40u;           
    uint16_t reg  = code << 5u;
    BQ25628E_WriteReg16(BQ25628E_REG_VINDPM, reg);
}

void BQ25628E_Set_VSYSMIN_mV(uint16_t voltage_mV) {
    if (voltage_mV < 2560) voltage_mV = 2560;
    if (voltage_mV > 3840) voltage_mV = 3840;
    uint16_t code = voltage_mV / 80u;          
    uint16_t reg  = code << 6u;
    BQ25628E_WriteReg16(BQ25628E_REG_VSYSMIN, reg);
}

void BQ25628E_Set_ChargerEnable(bool enable) {
    uint8_t val = enable ? BQ25628E_CTRL0_EN_CHG : 0;
    BQ25628E_UpdateBits8(BQ25628E_REG_CTRL0, BQ25628E_CTRL0_EN_CHG, val);
}

void BQ25628E_Set_HIZ(bool enable) {
    uint8_t val = enable ? BQ25628E_CTRL0_EN_HIZ : 0;
    BQ25628E_UpdateBits8(BQ25628E_REG_CTRL0, BQ25628E_CTRL0_EN_HIZ, val);
}

void BQ25628E_Set_Precharge_mA(uint16_t current_mA) {
    if (current_mA < 10)   current_mA = 10;
    if (current_mA > 310) current_mA = 310;
    uint16_t code = current_mA / 10u;    
    uint16_t reg  = code << 3u;
    BQ25628E_WriteReg8(BQ25628E_REG_IPRECHG, reg);
}

void BQ25628E_Set_Termination_mA(uint16_t current_mA) {
    if (current_mA < 5)   current_mA = 5;
    if (current_mA > 310) current_mA = 310;
    uint16_t code = current_mA / 5u;    
    uint16_t reg  = code << 2u;
    BQ25628E_WriteReg8(BQ25628E_REG_TERM_CTRL, reg);
}

void BQ25628E_Set_TS_Ignore(bool ignore) {
    uint8_t val = ignore ? BQ25628E_NTC0_TS_IGNORE : 0u;
    BQ25628E_UpdateBits8(BQ25628E_REG_NTC0, BQ25628E_NTC0_TS_IGNORE, val);
}

void BQ25628E_Set_PeakDischarge_6A(void) {
    BQ25628E_UpdateBits8(BQ25628E_REG_CTRL3,
                         BQ25628E_CTRL3_IBAT_PK_MASK,
                         BQ25628E_CTRL3_IBAT_PK_6A);
}


void BQ25628E_Disable_Watchdog(void) {
    // We update bits 1:0 to 00b to disable the timer
    BQ25628E_UpdateBits8(BQ25628E_REG_CTRL0, 
                         BQ25628E_CTRL0_WATCHDOG_MASK, 
                         BQ25628E_CTRL0_WATCHDOG_DIS);
}

void BQ25628E_ADC_Control(bool enable) {
    if (enable) {
        BQ25628E_WriteReg8(BQ25628E_REG_ADC_CTRL, BQ25628E_ADC_EN);
        delay_cycles(64000000); 
    } else {
        BQ25628E_WriteReg8(BQ25628E_REG_ADC_CTRL, 0x00);
    }
}

/* Getters unchanged */
/* --- Getters --- */
uint16_t BQ25628E_Get_VBAT_mV(void) { return g_vbat_mV; }
uint16_t BQ25628E_Get_VSYS_mV(void) { return g_vsys_mV; }
uint16_t BQ25628E_Get_VBUS_mV(void) { return g_vbus_mV; }
int16_t  BQ25628E_Get_IBUS_mA(void) { return g_ibus_mA; }
int16_t  BQ25628E_Get_IBAT_mA(void) { return g_ibat_mA; }
int16_t  BQ25628E_Get_TDIE_C(void) { return g_tdie_C; }
float BQ25628E_Get_TBAT_C(void) { return g_tbat_C; }
uint8_t BQ25628E_GetFaultFlags(void) {
    return BQ25628E_ReadReg8(BQ25628E_REG_FAULT_FLAG0);
}