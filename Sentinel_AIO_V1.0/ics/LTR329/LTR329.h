#ifndef LTR329_H
#define LTR329_H

#include <stdint.h>
#include <stdbool.h>
#include "HAL/i2c.h"

/* I2C Address */
#define LTR329_I2C_ADDR         0x29

/* Register Addresses */
#define LTR329_REG_ALS_CONTR    0x80
#define LTR329_REG_ALS_MEAS_RATE 0x85
#define LTR329_REG_PART_ID      0x86
#define LTR329_REG_MANUFAC_ID   0x87
#define LTR329_REG_DATA_CH1_0   0x88
#define LTR329_REG_DATA_CH1_1   0x89
#define LTR329_REG_DATA_CH0_0   0x8A
#define LTR329_REG_DATA_CH0_1   0x8B
#define LTR329_REG_ALS_STATUS   0x8C

/* ALS_CONTR bits */
#define LTR329_CONTR_GAIN_1X    (0x00 << 2)
#define LTR329_CONTR_GAIN_2X    (0x01 << 2)
#define LTR329_CONTR_GAIN_4X    (0x02 << 2)
#define LTR329_CONTR_GAIN_8X    (0x03 << 2)
#define LTR329_CONTR_GAIN_48X   (0x06 << 2)
#define LTR329_CONTR_GAIN_96X   (0x07 << 2)
#define LTR329_CONTR_SW_RESET   (1 << 1)
#define LTR329_CONTR_ACTIVE     (1 << 0)
#define LTR329_CONTR_STANDBY    (0 << 0)

/* ALS_STATUS bits */
#define LTR329_STATUS_INVALID   (1 << 7)
#define LTR329_STATUS_NEW_DATA  (1 << 2)

typedef enum {
    LTR329_GAIN_1X = 1,
    LTR329_GAIN_2X = 2,
    LTR329_GAIN_4X = 4,
    LTR329_GAIN_8X = 8,
    LTR329_GAIN_48X = 48,
    LTR329_GAIN_96X = 96
} LTR329_Gain;

typedef enum {
    LTR329_INT_50MS = 50,
    LTR329_INT_100MS = 100,
    LTR329_INT_150MS = 150,
    LTR329_INT_200MS = 200,
    LTR329_INT_250MS = 250,
    LTR329_INT_300MS = 300,
    LTR329_INT_350MS = 350,
    LTR329_INT_400MS = 400
} LTR329_IntegrationTime;

/* Driver Structure */
typedef struct {
    I2C_Regs *i2c;
    LTR329_Gain gain;
    LTR329_IntegrationTime int_time;
    bool initialized;
} LTR329_Handle;

extern LTR329_Handle gLTR329;

/* Public API */
bool LTR329_Init(I2C_Regs *i2c);
bool LTR329_SetGain(LTR329_Gain gain);
bool LTR329_SetTiming(LTR329_IntegrationTime int_time, uint16_t meas_rate_ms);
bool LTR329_ReadData(uint16_t *ch0, uint16_t *ch1);
bool LTR329_GetStatus(uint8_t *status);
float LTR329_CalculateLux(uint16_t ch0, uint16_t ch1);

#endif /* LTR329_H */
