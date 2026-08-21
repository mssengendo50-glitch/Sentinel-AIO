#include "LTR329.h"
#include <math.h>
#include "HAL/uart.h"

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

    /* 3. Set Active Mode and Default Gain.
     *
     * Active, not standby. The AE seed needs a completed conversion within
     * ~140 ms of the STM32 rail coming up, and the part only starts its 10 ms
     * wake + integration cycle once it is active. SM_HandleState_POWER_STM()
     * does call LTR329_SetMode(true) on wake, so this is belt and braces - but
     * it also covers the CLI and any path that reads the ALS without going
     * through the state machine. Idle current is reclaimed by
     * SM_SetSTMPower(false), which puts it back in standby. */
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
        /* ################ THIS LINE IS WRONG. DO NOT "TIDY" IT. ############
         *
         * It should be   0.5926f * ch0 + 0.1185f * ch1
         * It currently is 5.9260f * ch0 - 0.1185f * ch1     (10x, sign flipped)
         *
         * It is left wrong ON PURPOSE, for now, because the decision trees in
         * als_model_separate.c were trained on lux produced by this exact
         * expression. Correcting the maths without retraining makes the seed
         * worse, not better. See docs/ALS_LUX_CORRECTION.md - read it before
         * touching this line.
         *
         * EVIDENCE (2026-08-21), three independent lines:
         *
         * 1. Lite-On Appendix A for LTR-303ALS/LTR-329ALS states
         *      ALS_LUX = (0.5926 * CH0 + 0.1185 * CH1) / ALS_GAIN / ALS_INT
         *    for 0.64 <= RATIO < 0.85. The ESPHome ltr_als_ps reference agrees
         *    verbatim. (The LTR-329 datasheet itself only points at Appendix A
         *    and does not contain the formula - which is how this got missed.)
         *
         * 2. Continuity. A piecewise lux formula must be continuous at its own
         *    boundaries. Expressed as a coefficient on ch0 at the boundary
         *    ratios, the real formula is - to four decimals:
         *        ratio 0.45:  below 2.6791   above 2.6791    continuous
         *        ratio 0.64:  below 0.8033   above 0.8033    continuous
         *    This line gives 5.7153 above 0.64: a 7.11x cliff.
         *
         * 3. Hardware. Two logged readings of the same scene, minutes apart:
         *        ch0=233 ch1=407  ratio 0.6359  ->  4.193 lux
         *        ch0=233 ch1=419  ratio 0.6426  -> 27.731 lux
         *    Identical ch0, twelve counts of ch1, 6.6x apart, because the ratio
         *    stepped over 0.64. Indoor tungsten sits at ratio ~0.64 on this
         *    part, so this is where the device actually lives - one ADC count
         *    of noise swings the AE seed by 7x.
         *
         * WHY IT CANNOT JUST BE FIXED. The data-collection firmware
         * (Test firmware/TEST) carries this same expression, so every training
         * label was computed with it. Worse, the 7.11x fold maps a genuinely
         * ~7x darker population onto the same lux values as the branch below,
         * so the training set contains contradictory labels at the same input.
         * The trees cannot be repaired by relabelling; the data has to be
         * recollected.
         *
         * THE FIX, IN ORDER:
         *   1. correct this line to the Appendix A form
         *   2. recollect with auto-ranging enabled and raw ch0/ch1/gain/int
         *      logged alongside lux (TEST collected at a fixed 1X/100 ms,
         *      which put every scene below ~5 lux inside the 0-6 count dark
         *      band - the low half of the set is quantisation noise)
         *   3. retrain als_model_separate.c
         *   4. only then shorten MIN_FRAMES_SEEDED on the STM32
         * ################################################################## */
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

/* Bench helper: prints "exposure, gain, lux" for a measured lux value, in the
 * CSV shape the model's training data used, so predictions can be logged
 * straight from the device and compared against captures. Used by `ltr model`
 * in functions.c. */
void LTR329_PrintPredictedExposureGain(float lux) {
    double input[1];
    input[0] = log1p((double)lux);

    double exp_pred = score_exposure_sep(input);
    double gain_pred = score_gain_sep(input);

    uart_printf("%ld, %ld, %ld\r\n", (int32_t)exp_pred, (int32_t)gain_pred, (int32_t)lux);
}

bool LTR329_SetMode(bool active) {
    if (!gLTR329.initialized) return false;
    
    uint8_t reg_val;
    if (I2C_ReadDevice(gLTR329.i2c, LTR329_I2C_ADDR, LTR329_REG_ALS_CONTR, &reg_val, 1) != I2C_SUCCESS) {
        return false;
    }
    
    if (active) {
        reg_val |= LTR329_CONTR_ACTIVE;
    } else {
        reg_val &= ~LTR329_CONTR_ACTIVE;
    }
    
    return I2C_WriteDevice(gLTR329.i2c, LTR329_I2C_ADDR, LTR329_REG_ALS_CONTR, &reg_val, 1) == I2C_SUCCESS;
}
