#ifndef ZDP323B_H_
#define ZDP323B_H_

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "HAL/i2c.h"
// ─────────────────────────────────────────────
// ZDP323B State
// ─────────────────────────────────────────────

// Filter step encoding per Table 4
// Step 1 = 01, Step 2 = 11, Step 3 = 00
typedef enum {
    ZDP323B_FILTER_STEP_1 = 0b01,
    ZDP323B_FILTER_STEP_2 = 0b11,
    ZDP323B_FILTER_STEP_3 = 0b00,
} ZDP323B_FilterStep;

// Filter type encoding per Table 4
// Type A = 111, Type B = 000, Type C = 001, Type D = 010, Direct = 011
typedef enum {
    ZDP323B_FILTER_TYPE_A      = 0b111,
    ZDP323B_FILTER_TYPE_B      = 0b000,
    ZDP323B_FILTER_TYPE_C      = 0b001,
    ZDP323B_FILTER_TYPE_D      = 0b010,
    ZDP323B_FILTER_TYPE_DIRECT = 0b011,
} ZDP323B_FilterType;

typedef struct {
    uint8_t          threshold;   // DETLVL[7:0], 0-255, actual = value * 8
    bool             trigger_en;  // TRIGOM, B23
    ZDP323B_FilterStep filter_step;
    ZDP323B_FilterType filter_type;
} ZDP323B_Config;

typedef struct {
    I2C_Regs          *i2c;
    uint16_t           dev_addr;
    ZDP323B_Config     armed_cfg;
    volatile bool      motion_detected;
    bool               initialized;
} ZDP323B_Device;

// Global instance - expand to array if multi-device needed later
extern ZDP323B_Device gPIR;

bool I2C_TryAddress10(I2C_Regs *bus, uint16_t addr);

I2C_Status ZDP323B_ReadPeakHold(I2C_Regs *i2c, uint16_t dev_addr, int16_t *peak_out);

I2C_Status ZDP323B_WriteConfig(I2C_Regs *i2c, uint16_t dev_addr, uint8_t *config_bytes);

void ZDP323B_MotionISR(void);
I2C_Status ZDP323B_Init(I2C_Regs *i2c, uint16_t dev_addr,
                         ZDP323B_FilterStep step, ZDP323B_FilterType type,
                         uint8_t threshold);

void ZDP323B_BuildConfigBytes(const ZDP323B_Config *cfg, uint8_t *out) ;

#endif /* ZDP323B_H_ */