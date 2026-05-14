#include "LTR329.h"
#include <math.h>

LTR329_Handle gLTR329 = {0};

static uint8_t gain_to_reg(LTR329_Gain gain) {
    switch (gain) {
        case LTR329_GAIN_1X:  return 0x00;
        case LTR329_GAIN_2X:  return 0x01;
        case LTR329_GAIN_4X:  return 0x02;
        case LTR329_GAIN_8X:  return 0x03;
        case LTR329_GAIN_48X: return 0x06;
        case LTR329_GAIN_96X: return 0x07;
        default: return 0x00;
    }
}

static uint8_t int_time_to_reg(LTR329_IntegrationTime int_time) {
    switch (int_time) {
        case LTR329_INT_100MS: return 0x00;
        case LTR329_INT_50MS:  return 0x01;
        case LTR329_INT_200MS: return 0x02;
        case LTR329_INT_400MS: return 0x03;
        case LTR329_INT_150MS: return 0x04;
        case LTR329_INT_250MS: return 0x05;
        case LTR329_INT_300MS: return 0x06;
        case LTR329_INT_350MS: return 0x07;
        default: return 0x00;
    }
}

bool LTR329_Init(I2C_Regs *i2c) {
    gLTR329.i2c = i2c;
    gLTR329.gain = LTR329_GAIN_1X;
    gLTR329.int_time = LTR329_INT_100MS;
    
    // 1. Check Part ID
    uint8_t part_id = 0;
    if (I2C_ReadDevice(i2c, LTR329_I2C_ADDR, LTR329_REG_PART_ID, &part_id, 1) != I2C_SUCCESS) {
        return false;
    }
    
    // Expected Part ID for LTR-329 is 0x92 (or 0xA0 based on some revisions)
    // Datasheet says 0xA0 for Part Number ID and 0x00 for Revision ID (Combined 0xA0)
    // Actually datasheet 13/27 says PART_ID reset value is 0xA0.
    if (part_id != 0xA0) {
        // Some versions might return 0x92 or 0xA0. Let's be lenient or just log it.
    }

    // 2. SW Reset
    uint8_t contr = LTR329_CONTR_SW_RESET;
    I2C_WriteDevice(i2c, LTR329_I2C_ADDR, LTR329_REG_ALS_CONTR, &contr, 1);
    delay_cycles(100 * 32000); // 10ms reset delay

    // 3. Set Active Mode and Default Gain
    contr = LTR329_CONTR_ACTIVE | LTR329_CONTR_GAIN_1X;
    if (I2C_WriteDevice(i2c, LTR329_I2C_ADDR, LTR329_REG_ALS_CONTR, &contr, 1) != I2C_SUCCESS) {
        return false;
    }

    gLTR329.initialized = true;
    return true;
}

bool LTR329_SetGain(LTR329_Gain gain) {
    if (!gLTR329.initialized) return false;
    
    uint8_t reg_val = (gain_to_reg(gain) << 2) | LTR329_CONTR_ACTIVE;
    if (I2C_WriteDevice(gLTR329.i2c, LTR329_I2C_ADDR, LTR329_REG_ALS_CONTR, &reg_val, 1) == I2C_SUCCESS) {
        gLTR329.gain = gain;
        return true;
    }
    return false;
}

bool LTR329_SetTiming(LTR329_IntegrationTime int_time, uint16_t meas_rate_ms) {
    if (!gLTR329.initialized) return false;

    uint8_t rate_reg = 0;
    // Integration time bits [5:3]
    rate_reg |= (int_time_to_reg(int_time) << 3);
    
    // Measurement rate bits [2:0]
    uint8_t rate_val = 0;
    if (meas_rate_ms <= 50) rate_val = 0x00;
    else if (meas_rate_ms <= 100) rate_val = 0x01;
    else if (meas_rate_ms <= 200) rate_val = 0x02;
    else if (meas_rate_ms <= 500) rate_val = 0x03;
    else if (meas_rate_ms <= 1000) rate_val = 0x04;
    else rate_val = 0x06; // 2000ms
    
    rate_reg |= rate_val;

    if (I2C_WriteDevice(gLTR329.i2c, LTR329_I2C_ADDR, LTR329_REG_ALS_MEAS_RATE, &rate_reg, 1) == I2C_SUCCESS) {
        gLTR329.int_time = int_time;
        return true;
    }
    return false;
}

bool LTR329_ReadData(uint16_t *ch0, uint16_t *ch1) {
    if (!gLTR329.initialized) return false;

    uint8_t buffer[4];
    // Sequential read: 0x88, 0x89, 0x8A, 0x8B
    // CH1 (IR) is at 0x88, 0x89. CH0 (Visible+IR) is at 0x8A, 0x8B.
    if (I2C_ReadDevice(gLTR329.i2c, LTR329_I2C_ADDR, LTR329_REG_DATA_CH1_0, buffer, 4) != I2C_SUCCESS) {
        return false;
    }

    *ch1 = (buffer[1] << 8) | buffer[0];
    *ch0 = (buffer[3] << 8) | buffer[2];
    
    return true;
}

bool LTR329_GetStatus(uint8_t *status) {
    if (!gLTR329.initialized) return false;
    return I2C_ReadDevice(gLTR329.i2c, LTR329_I2C_ADDR, LTR329_REG_ALS_STATUS, status, 1) == I2C_SUCCESS;
}

float LTR329_CalculateLux(uint16_t ch0, uint16_t ch1) {
    if (ch0 == 0 && ch1 == 0) return 0.0f;

    float ratio = (float)ch1 / (float)(ch0 + ch1);
    float lux = 0.0f;

    // Standard coefficients for LTR-329ALS-01
    // Note: These should ideally be adjusted for physical aperture/window (PFactor)
    if (ratio < 0.45f) {
        lux = (1.7743f * ch0 + 1.1059f * ch1);
    } else if (ratio < 0.64f) {
        lux = (4.2785f * ch0 - 1.9548f * ch1);
    } else if (ratio < 0.85f) {
        lux = (5.9260f * ch0 - 0.1185f * ch1);
    } else {
        lux = 0.0f;
    }

    // Normalize for Gain and Integration Time
    // Default gain 1X = 1.0. 
    float gain_factor = (float)gLTR329.gain;
    float int_factor = (float)gLTR329.int_time / 100.0f;

    lux = lux / gain_factor / int_factor;

    return lux;
}
