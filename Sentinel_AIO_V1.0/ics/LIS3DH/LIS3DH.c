#include "LIS3DH.h"

LIS3DH_Handle gLIS3DH = {0};

bool LIS3DH_Init(I2C_Regs *i2c, uint8_t addr) {
    gLIS3DH.i2c = i2c;
    gLIS3DH.dev_addr = addr;
    gLIS3DH.initialized = false;

    // 1. Check WHO_AM_I
    uint8_t whoami = 0;
    if (I2C_ReadDevice(i2c, addr, LIS3DH_REG_WHO_AM_I, &whoami, 1) != I2C_SUCCESS) {
        return false;
    }

    if (whoami != LIS3DH_WHO_AM_I_VALUE) {
        return false;
    }

    // 2. Default Configuration: 50Hz, all axes enabled, normal mode
    uint8_t cr1 = (LIS3DH_ODR_50HZ << 4) | 0x07; // 0x47
    if (I2C_WriteDevice(i2c, addr, LIS3DH_REG_CTRL_REG1, &cr1, 1) != I2C_SUCCESS) {
        return false;
    }

    // 3. Set range ±2g, High Resolution (HR) disabled, Block Data Update (BDU) enabled
    uint8_t cr4 = 0x80; // BDU = 1, FS = 2g, HR = 0
    if (I2C_WriteDevice(i2c, addr, LIS3DH_REG_CTRL_REG4, &cr4, 1) != I2C_SUCCESS) {
        return false;
    }

    gLIS3DH.range = LIS3DH_RANGE_2G;
    gLIS3DH.initialized = true;
    return true;
}

bool LIS3DH_SetODR(LIS3DH_ODR odr) {
    if (!gLIS3DH.initialized) return false;

    uint8_t cr1 = 0;
    if (I2C_ReadDevice(gLIS3DH.i2c, gLIS3DH.dev_addr, LIS3DH_REG_CTRL_REG1, &cr1, 1) != I2C_SUCCESS) {
        return false;
    }

    cr1 &= 0x0F; // Clear ODR bits
    cr1 |= (odr << 4);

    return I2C_WriteDevice(gLIS3DH.i2c, gLIS3DH.dev_addr, LIS3DH_REG_CTRL_REG1, &cr1, 1) == I2C_SUCCESS;
}

bool LIS3DH_SetRange(LIS3DH_Range range) {
    if (!gLIS3DH.initialized) return false;

    uint8_t cr4 = 0;
    if (I2C_ReadDevice(gLIS3DH.i2c, gLIS3DH.dev_addr, LIS3DH_REG_CTRL_REG4, &cr4, 1) != I2C_SUCCESS) {
        return false;
    }

    cr4 &= 0xCF; // Clear FS bits [5:4]
    cr4 |= (range << 4);

    if (I2C_WriteDevice(gLIS3DH.i2c, gLIS3DH.dev_addr, LIS3DH_REG_CTRL_REG4, &cr4, 1) == I2C_SUCCESS) {
        gLIS3DH.range = range;
        return true;
    }
    return false;
}

bool LIS3DH_ReadRaw(int16_t *x, int16_t *y, int16_t *z) {
    if (!gLIS3DH.initialized) return false;

    uint8_t buffer[6];
    // Use bit 7 for auto-increment in I2C
    if (I2C_ReadDevice(gLIS3DH.i2c, gLIS3DH.dev_addr, LIS3DH_REG_OUT_X_L | 0x80, buffer, 6) != I2C_SUCCESS) {
        return false;
    }

    *x = (int16_t)((buffer[1] << 8) | buffer[0]);
    *y = (int16_t)((buffer[3] << 8) | buffer[2]);
    *z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return true;
}

bool LIS3DH_ReadMg(float *x_mg, float *y_mg, float *z_mg) {
    int16_t rx, ry, rz;
    if (!LIS3DH_ReadRaw(&rx, &ry, &rz)) return false;

    float sensitivity = 1.0f; // mg/LSB for 10-bit normal mode at 2g
    // In Normal Mode (10-bit):
    // 2g: 4 mg/digit (Actually datasheet says 4mg/digit for Normal mode)
    // 4g: 8 mg/digit
    // 8g: 16 mg/digit
    // 16g: 48 mg/digit
    // Wait, the data is left-justified in 16 bits.
    // 10-bit data in 16-bit register means we divide by 64 (2^6).
    
    switch (gLIS3DH.range) {
        case LIS3DH_RANGE_2G:  sensitivity = 4.0f; break;
        case LIS3DH_RANGE_4G:  sensitivity = 8.0f; break;
        case LIS3DH_RANGE_8G:  sensitivity = 16.0f; break;
        case LIS3DH_RANGE_16G: sensitivity = 48.0f; break;
        default: sensitivity = 4.0f; break;
    }

    // Since data is left-justified, we shift right by 6 to get the 10-bit value
    *x_mg = (float)(rx >> 6) * sensitivity;
    *y_mg = (float)(ry >> 6) * sensitivity;
    *z_mg = (float)(rz >> 6) * sensitivity;

    return true;
}
