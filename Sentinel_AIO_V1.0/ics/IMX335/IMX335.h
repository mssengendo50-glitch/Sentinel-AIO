#ifndef IMX335_H
#define IMX335_H

#include <stdint.h>
#include <stdbool.h>
#include "HAL/i2c.h"

/* Possible 7-bit I2C Slave Addresses (depends on ADDR pin state) */
#define IMX335_I2C_ADDR_0         0x1A
#define IMX335_I2C_ADDR_1         0x36

/* Main Registers */
#define IMX335_REG_MODE_SELECT    0x3000
#define IMX335_REG_HOLD           0x3001
#define IMX335_REG_VMAX           0x3030
#define IMX335_REG_SHUTTER        0x3058
#define IMX335_REG_GAIN           0x30e8
#define IMX335_REG_TPG            0x329e
#define IMX335_REG_ID             0x3912

#define IMX335_REG_HREVERSE       0x304E
#define IMX335_REG_VREVERSE       0x304F
#define AREA3_ST_ADR_1_LSB        0x3074
#define AREA3_ST_ADR_1_MSB        0x3075

/* Operational Modes */
#define IMX335_MODE_STREAMING     0x00
#define IMX335_MODE_STANDBY       0x01
#define IMX335_CHIP_ID            0x00

/* Constants */
#define IMX335_SHUTTER_MIN        9
#define IMX335_EXPOSURE_DEFAULT   23814
#define IMX335_NAME               "IMX335"
#define IMX335_BAYER_PATTERN      0   /* RGGB */
#define IMX335_COLOR_DEPTH        10  /* bits */
#define IMX335_GAIN_MIN           0   /* mdB */
#define IMX335_GAIN_MAX           72000 /* mdB */
#define IMX335_GAIN_DEFAULT       20000 /* mdB */
#define IMX335_GAIN_UNIT_MDB      300   /* 0.3 dB steps */
#define IMX335_EXPOSURE_MIN       0     /* us */
#define IMX335_EXPOSURE_MAX       33266 /* us */

#define IMX335_WIDTH              2592
#define IMX335_HEIGHT             1944

/* Driver Structure */
typedef struct {
    I2C_Regs *i2c;
    uint8_t dev_addr;
    bool initialized;
} IMX335_Handle;

extern IMX335_Handle gIMX335;

/* Public API */
bool IMX335_Init(I2C_Regs *i2c);
bool IMX335_ReadID(uint32_t *id);
bool IMX335_ReadReg(uint16_t reg, uint8_t *val);
bool IMX335_WriteReg(uint16_t reg, uint8_t val);
bool IMX335_SetGain(uint32_t gain_mdB);
bool IMX335_SetExposure(uint32_t exposure_us);
bool IMX335_SetTestPattern(int32_t mode);
bool IMX335_Scan(void);
bool IMX335_Start(void);
bool IMX335_Stop(void);

#endif /* IMX335_H */
