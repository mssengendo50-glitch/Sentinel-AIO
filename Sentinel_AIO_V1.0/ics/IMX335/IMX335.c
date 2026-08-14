#include "IMX335.h"
#include <string.h>

IMX335_Handle gIMX335 = {0};



/* Internal structure to represent registers */
struct regval {
    uint16_t addr;
    uint8_t val;
};

/* Sony IMX335 2592x1944 Register List */
static const struct regval res_2592_1944_regs[] = {
    {0x3000, 0x01},
    {0x3002, 0x00},
    {0x300c, 0x3b},
    {0x300d, 0x2a},
    {0x3018, 0x04},
    {0x302c, 0x3c},
    {0x302e, 0x20},
    {0x3056, 0x98},
    {0x3074, 0xc8},
    {0x3076, 0x30},
    {0x304c, 0x00},
    {0x314c, 0xc6},
    {0x315a, 0x02},
    {0x3168, 0xa0},
    {0x316a, 0x7e},
    {0x31a1, 0x00},
    {0x3288, 0x21},
    {0x328a, 0x02},
    {0x3414, 0x05},
    {0x3416, 0x18},
    {0x3648, 0x01},
    {0x364a, 0x04},
    {0x364c, 0x04},
    {0x3678, 0x01},
    {0x367c, 0x31},
    {0x367e, 0x31},
    {0x3706, 0x10},
    {0x3708, 0x03},
    {0x3714, 0x02},
    {0x3715, 0x02},
    {0x3716, 0x01},
    {0x3717, 0x03},
    {0x371c, 0x3d},
    {0x371d, 0x3f},
    {0x372c, 0x00},
    {0x372d, 0x00},
    {0x372e, 0x46},
    {0x372f, 0x00},
    {0x3730, 0x89},
    {0x3731, 0x00},
    {0x3732, 0x08},
    {0x3733, 0x01},
    {0x3734, 0xfe},
    {0x3735, 0x05},
    {0x3740, 0x02},
    {0x375d, 0x00},
    {0x375e, 0x00},
    {0x375f, 0x11},
    {0x3760, 0x01},
    {0x3768, 0x1b},
    {0x3769, 0x1b},
    {0x376a, 0x1b},
    {0x376b, 0x1b},
    {0x376c, 0x1a},
    {0x376d, 0x17},
    {0x376e, 0x0f},
    {0x3776, 0x00},
    {0x3777, 0x00},
    {0x3778, 0x46},
    {0x3779, 0x00},
    {0x377a, 0x89},
    {0x377b, 0x00},
    {0x377c, 0x08},
    {0x377d, 0x01},
    {0x377e, 0x23},
    {0x377f, 0x02},
    {0x3780, 0xd9},
    {0x3781, 0x03},
    {0x3782, 0xf5},
    {0x3783, 0x06},
    {0x3784, 0xa5},
    {0x3788, 0x0f},
    {0x378a, 0xd9},
    {0x378b, 0x03},
    {0x378c, 0xeb},
    {0x378d, 0x05},
    {0x378e, 0x87},
    {0x378f, 0x06},
    {0x3790, 0xf5},
    {0x3792, 0x43},
    {0x3794, 0x7a},
    {0x3796, 0xa1},
    {0x37b0, 0x36},
    {0x3a00, 0x01},
};

/* 2-Lane MIPI CSI-2 10-Bit configuration */
static const struct regval mode_2l_10b_regs[] = {
    {0x3050, 0x00},
    {0x319D, 0x00},
    {0x341c, 0xff},
    {0x341d, 0x01},
    {0x3a01, 0x01},
};

/* Test Pattern Generator Enable */
static const struct regval test_pattern_enable_regs[] = {
    {0x3148, 0x10},
    {0x3280, 0x00},
    {0x329c, 0x01},
    {0x32a0, 0x11},
    {0x3302, 0x00},
    {0x3303, 0x00},
    {0x336c, 0x00},
};

/* Test Pattern Generator Disable */
static const struct regval test_pattern_disable_regs[] = {
    {0x3148, 0x00},
    {0x3280, 0x01},
    {0x329c, 0x00},
    {0x32a0, 0x10},
    {0x3302, 0x32},
    {0x3303, 0x00},
    {0x336c, 0x01},
};

/* -------------------------------------------------------------------------- */
/* Low-level I2C Helpers with 16-bit Sub-addressing                           */
/* -------------------------------------------------------------------------- */

static I2C_Status I2C_WriteDevice16(I2C_Regs *i2c, uint8_t dev_addr, uint16_t reg_addr, 
                                    uint8_t *reg_data, uint8_t count) {
    extern void PIR_Interrupt_PauseForI2C(void);
    extern void PIR_Interrupt_ResumeAfterI2C(void);

    if (i2c == I2C_0_INST) {
        PIR_Interrupt_PauseForI2C();
    }

    gTxPacket[0] = (uint8_t)(reg_addr >> 8);
    gTxPacket[1] = (uint8_t)(reg_addr & 0xFF);
    for (int i = 0; i < count; i++) {
        gTxPacket[i+2] = reg_data[i];
    }

    DL_I2C_flushControllerTXFIFO(i2c);
    DL_I2C_fillControllerTXFIFO(i2c, &gTxPacket[0], count + 2);
    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);

    while (!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE));

    gI2cControllerStatus = I2C_STATUS_TX_STARTED;
    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, count + 2);

    while ((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) && 
           (gI2cControllerStatus != I2C_STATUS_ERROR)) {
        __WFE();
    }

    if (gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(i2c);
        if (i2c == I2C_0_INST) {
            PIR_Interrupt_ResumeAfterI2C();
        }
        return I2C_ERROR_NACK; 
    }

    while (DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    
    delay_cycles(1000);
    DL_I2C_flushControllerTXFIFO(i2c);
    
    if (i2c == I2C_0_INST) {
        PIR_Interrupt_ResumeAfterI2C();
    }
    return I2C_SUCCESS;
}

static I2C_Status I2C_ReadDevice16(I2C_Regs *i2c, uint8_t dev_addr, uint16_t reg_addr, 
                                   uint8_t *reg_data, uint8_t count) {
    extern void PIR_Interrupt_PauseForI2C(void);
    extern void PIR_Interrupt_ResumeAfterI2C(void);

    if (i2c == I2C_0_INST) {
        PIR_Interrupt_PauseForI2C();
    }

    gRxLen   = count;
    gRxCount = 0;

    gI2cControllerStatus = I2C_STATUS_RX_STARTED;

    // Send 16-bit register address (TX)
    gTxPacket[0] = (uint8_t)(reg_addr >> 8);
    gTxPacket[1] = (uint8_t)(reg_addr & 0xFF);
    DL_I2C_flushControllerTXFIFO(i2c);
    DL_I2C_fillControllerTXFIFO(i2c, gTxPacket, 2);
    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);

    while (!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    while ((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) &&
           (gI2cControllerStatus != I2C_STATUS_ERROR)) {
        __WFE();
    }

    if (gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(i2c);
        if (i2c == I2C_0_INST) {
            PIR_Interrupt_ResumeAfterI2C();
        }
        return I2C_ERROR_NACK;
    }

    while (DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    delay_cycles(1000);

    // Read data (RX)
    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
                               DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, count);

    while ((gI2cControllerStatus != I2C_STATUS_RX_COMPLETE) &&
           (gI2cControllerStatus != I2C_STATUS_ERROR)) {
        __WFE();
    }

    if (gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerRXFIFO(i2c);
        if (i2c == I2C_0_INST) {
            PIR_Interrupt_ResumeAfterI2C();
        }
        return I2C_ERROR_NACK;
    }

    for (uint8_t i = 0; i < count; i++) {
        reg_data[i] = gRxPacket[i];
    }

    DL_I2C_flushControllerTXFIFO(i2c);
    DL_I2C_flushControllerRXFIFO(i2c);

    if (i2c == I2C_0_INST) {
        PIR_Interrupt_ResumeAfterI2C();
    }
    return I2C_SUCCESS;
}

/* -------------------------------------------------------------------------- */
/* Public API Implementation                                                  */
/* -------------------------------------------------------------------------- */

static bool IMX335_WriteTable(const struct regval *regs, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        uint8_t val = regs[i].val;
        if (I2C_WriteDevice16(gIMX335.i2c, gIMX335.dev_addr, regs[i].addr, &val, 1) != I2C_SUCCESS) {
            return false;
        }
    }
    return true;
}

bool IMX335_Init(I2C_Regs *i2c) {
    gIMX335.i2c = i2c;
    gIMX335.initialized = false;

    // Detect I2C Address (defaulting to 0x1A or 0x36)
    if (I2C_TryAddress(i2c, IMX335_I2C_ADDR_0)) {
        gIMX335.dev_addr = IMX335_I2C_ADDR_0;
    } else if (I2C_TryAddress(i2c, IMX335_I2C_ADDR_1)) {
        gIMX335.dev_addr = IMX335_I2C_ADDR_1;
    } else {
        return false;
    }

    // Verify Sensor ID
    uint32_t id = 0;
    if (!IMX335_ReadID(&id) || id != IMX335_CHIP_ID) {
        return false;
    }

    // Write base resolution table
    if (!IMX335_WriteTable(res_2592_1944_regs, sizeof(res_2592_1944_regs)/sizeof(struct regval))) {
        return false;
    }

    // Write MIPI 2-lane 10-bit mode settings
    if (!IMX335_WriteTable(mode_2l_10b_regs, sizeof(mode_2l_10b_regs)/sizeof(struct regval))) {
        return false;
    }

    gIMX335.initialized = true;
    return true;
}

bool IMX335_ReadID(uint32_t *id) {
    uint8_t tmp = 0xFF;
    if (I2C_ReadDevice16(gIMX335.i2c, gIMX335.dev_addr, IMX335_REG_ID, &tmp, 1) != I2C_SUCCESS) {
        return false;
    }
    *id = tmp;
    return true;
}

bool IMX335_ReadReg(uint16_t reg, uint8_t *val) {
    return (I2C_ReadDevice16(gIMX335.i2c, gIMX335.dev_addr, reg, val, 1) == I2C_SUCCESS);
}

bool IMX335_WriteReg(uint16_t reg, uint8_t val) {
    return (I2C_WriteDevice16(gIMX335.i2c, gIMX335.dev_addr, reg, &val, 1) == I2C_SUCCESS);
}

bool IMX335_SetGain(uint32_t gain_mdB) {
    if (!gIMX335.initialized) return false;
    if (gain_mdB < IMX335_GAIN_MIN || gain_mdB > IMX335_GAIN_MAX) return false;

    uint32_t gain_steps = gain_mdB / IMX335_GAIN_UNIT_MDB;

    uint8_t hold = 1;
    IMX335_WriteReg(IMX335_REG_HOLD, hold);

    uint8_t data[2];
    data[0] = (uint8_t)(gain_steps & 0xFF);
    data[1] = (uint8_t)((gain_steps >> 8) & 0xFF);
    
    bool success = (I2C_WriteDevice16(gIMX335.i2c, gIMX335.dev_addr, IMX335_REG_GAIN, data, 2) == I2C_SUCCESS);

    hold = 0;
    IMX335_WriteReg(IMX335_REG_HOLD, hold);

    return success;
}

bool IMX335_SetExposure(uint32_t exposure_us) {
    if (!gIMX335.initialized) return false;
    if (exposure_us < IMX335_EXPOSURE_MIN || exposure_us > IMX335_EXPOSURE_MAX) return false;

    uint8_t vmax_data[4] = {0};
    if (I2C_ReadDevice16(gIMX335.i2c, gIMX335.dev_addr, IMX335_REG_VMAX, vmax_data, 4) != I2C_SUCCESS) {
        return false;
    }

    uint32_t vmax = (uint32_t)vmax_data[0] |
                    ((uint32_t)vmax_data[1] << 8) |
                    ((uint32_t)vmax_data[2] << 16) |
                    ((uint32_t)vmax_data[3] << 24);

    float line_period_us = 1000000.0f / (4500.0f * 30.0f); // ~7.4074 us
    uint32_t shutter = vmax - (uint32_t)((float)exposure_us / line_period_us);

    if (shutter < IMX335_SHUTTER_MIN) {
        return false;
    }

    uint8_t hold = 1;
    IMX335_WriteReg(IMX335_REG_HOLD, hold);

    uint8_t shutter_data[3];
    shutter_data[0] = (uint8_t)(shutter & 0xFF);
    shutter_data[1] = (uint8_t)((shutter >> 8) & 0xFF);
    shutter_data[2] = (uint8_t)((shutter >> 16) & 0xFF);

    bool success = (I2C_WriteDevice16(gIMX335.i2c, gIMX335.dev_addr, IMX335_REG_SHUTTER, shutter_data, 3) == I2C_SUCCESS);

    hold = 0;
    IMX335_WriteReg(IMX335_REG_HOLD, hold);

    return success;
}

bool IMX335_SetTestPattern(int32_t mode) {
    if (!gIMX335.initialized) return false;

    if (mode >= 0) {
        uint8_t val = (uint8_t)mode;
        if (I2C_WriteDevice16(gIMX335.i2c, gIMX335.dev_addr, IMX335_REG_TPG, &val, 1) != I2C_SUCCESS) {
            return false;
        }
        return IMX335_WriteTable(test_pattern_enable_regs, sizeof(test_pattern_enable_regs)/sizeof(struct regval));
    } else {
        return IMX335_WriteTable(test_pattern_disable_regs, sizeof(test_pattern_disable_regs)/sizeof(struct regval));
    }
}

bool IMX335_Scan(void) {
    if (I2C_TryAddress(I2C_1_INST, IMX335_I2C_ADDR_0)) {
        gIMX335.dev_addr = IMX335_I2C_ADDR_0;
        return true;
    }
    if (I2C_TryAddress(I2C_1_INST, IMX335_I2C_ADDR_1)) {
        gIMX335.dev_addr = IMX335_I2C_ADDR_1;
        return true;
    }
    return false;
}

bool IMX335_Start(void) {
    if (!gIMX335.initialized) return false;
    uint8_t stream_mode = IMX335_MODE_STREAMING;
    if (I2C_WriteDevice16(gIMX335.i2c, gIMX335.dev_addr, IMX335_REG_MODE_SELECT, &stream_mode, 1) != I2C_SUCCESS) {
        return false;
    }
    delay_cycles(20 * 32000); // 20ms settle delay
    return true;
}

bool IMX335_Stop(void) {
    if (!gIMX335.initialized) return false;
    uint8_t standby_mode = IMX335_MODE_STANDBY;
    return (I2C_WriteDevice16(gIMX335.i2c, gIMX335.dev_addr, IMX335_REG_MODE_SELECT, &standby_mode, 1) == I2C_SUCCESS);
}
