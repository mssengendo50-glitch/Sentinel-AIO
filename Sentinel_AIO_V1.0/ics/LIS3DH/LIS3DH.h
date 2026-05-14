#ifndef LIS3DH_H
#define LIS3DH_H

#include <stdint.h>
#include <stdbool.h>
#include "HAL/i2c.h"

/* I2C Addresses */
#define LIS3DH_I2C_ADDR_0       0x18  /* SA0 = GND */
#define LIS3DH_I2C_ADDR_1       0x19  /* SA0 = VDD */

/* Register Map */
#define LIS3DH_REG_STATUS_AUX   0x07
#define LIS3DH_REG_OUT_ADC1_L   0x08
#define LIS3DH_REG_OUT_ADC1_H   0x09
#define LIS3DH_REG_OUT_ADC2_L   0x0A
#define LIS3DH_REG_OUT_ADC2_H   0x0B
#define LIS3DH_REG_OUT_ADC3_L   0x0C
#define LIS3DH_REG_OUT_ADC3_H   0x0D
#define LIS3DH_REG_WHO_AM_I     0x0F
#define LIS3DH_REG_CTRL_REG0    0x1E
#define LIS3DH_REG_TEMP_CFG     0x1F
#define LIS3DH_REG_CTRL_REG1    0x20
#define LIS3DH_REG_CTRL_REG2    0x21
#define LIS3DH_REG_CTRL_REG3    0x22
#define LIS3DH_REG_CTRL_REG4    0x23
#define LIS3DH_REG_CTRL_REG5    0x24
#define LIS3DH_REG_CTRL_REG6    0x25
#define LIS3DH_REG_REFERENCE    0x26
#define LIS3DH_REG_STATUS       0x27
#define LIS3DH_REG_OUT_X_L      0x28
#define LIS3DH_REG_OUT_X_H      0x29
#define LIS3DH_REG_OUT_Y_L      0x2A
#define LIS3DH_REG_OUT_Y_H      0x2B
#define LIS3DH_REG_OUT_Z_L      0x2C
#define LIS3DH_REG_OUT_Z_H      0x2D
#define LIS3DH_REG_FIFO_CTRL    0x2E
#define LIS3DH_REG_FIFO_SRC     0x2F
#define LIS3DH_REG_INT1_CFG     0x30
#define LIS3DH_REG_INT1_SRC     0x31
#define LIS3DH_REG_INT1_THS     0x32
#define LIS3DH_REG_INT1_DUR     0x33

#define LIS3DH_WHO_AM_I_VALUE   0x33

/* Enums for configuration */
typedef enum {
    LIS3DH_ODR_POWER_DOWN = 0x00,
    LIS3DH_ODR_1HZ        = 0x01,
    LIS3DH_ODR_10HZ       = 0x02,
    LIS3DH_ODR_25HZ       = 0x03,
    LIS3DH_ODR_50HZ       = 0x04,
    LIS3DH_ODR_100HZ      = 0x05,
    LIS3DH_ODR_200HZ      = 0x06,
    LIS3DH_ODR_400HZ      = 0x07,
    LIS3DH_ODR_1620HZ_LP  = 0x08,
    LIS3DH_ODR_5376HZ_LP  = 0x09
} LIS3DH_ODR;

typedef enum {
    LIS3DH_RANGE_2G  = 0x00,
    LIS3DH_RANGE_4G  = 0x01,
    LIS3DH_RANGE_8G  = 0x02,
    LIS3DH_RANGE_16G = 0x03
} LIS3DH_Range;

/* Driver Handle */
typedef struct {
    I2C_Regs *i2c;
    uint8_t dev_addr;
    LIS3DH_Range range;
    bool initialized;
} LIS3DH_Handle;

extern LIS3DH_Handle gLIS3DH;

/* API Functions */
bool LIS3DH_Init(I2C_Regs *i2c, uint8_t addr);
bool LIS3DH_SetODR(LIS3DH_ODR odr);
bool LIS3DH_SetRange(LIS3DH_Range range);
bool LIS3DH_ReadRaw(int16_t *x, int16_t *y, int16_t *z);
bool LIS3DH_ReadMg(float *x_mg, float *y_mg, float *z_mg);

#endif /* LIS3DH_H */
