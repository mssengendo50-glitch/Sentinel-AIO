#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

/* * BQ7690x / BQ27Z7 I2C Functions Header File
 * Updated for Dual-Instance Support (I2C0 and I2C1)
 */

/********* I2C Master Driver Functions *********/

typedef enum {
    I2C_SUCCESS = 0,
    I2C_ERROR_NACK,
    I2C_ERROR_TIMEOUT,
    I2C_ERROR_ARB_LOST
} I2C_Status;

enum I2cControllerStatus {
    I2C_STATUS_IDLE = 0,
    I2C_STATUS_TX_STARTED,
    I2C_STATUS_TX_INPROGRESS,
    I2C_STATUS_TX_COMPLETE,
    I2C_STATUS_RX_STARTED,
    I2C_STATUS_RX_INPROGRESS,
    I2C_STATUS_RX_COMPLETE,
    I2C_STATUS_ERROR,
};

extern volatile enum I2cControllerStatus gI2cControllerStatus;

/* Counters and Buffers */
extern uint32_t gTxLen, gTxCount;
extern uint8_t gTxPacket[64];
extern uint8_t gRxPacket[64];
extern volatile uint32_t gRxLen, gRxCount;

/* ─────────────────────────────────────────────────────────────────────────────
 * Timeout configuration
 * ───────────────────────────────────────────────────────────────────────────*/
#define I2C_TIMEOUT_LOOPS       (200000UL)
#define I2C_RECOVERY_HALF_BIT   (160U)

#define I2C_WAIT_WHILE(cond, timed_out)                     \
    do {                                                    \
        uint32_t _to = I2C_TIMEOUT_LOOPS;                   \
        (timed_out) = false;                                \
        while (cond) {                                      \
            if (--_to == 0U) { (timed_out) = true; break; } \
        }                                                   \
    } while (0)

/* ─────────────────────────────────────────────────────────────────────────────
 * Peripheral power-state flags.
 * ───────────────────────────────────────────────────────────────────────────*/
extern volatile bool g_i2c0_powered;
extern volatile bool g_i2c1_powered;

void i2c_init(void);
// Shared logic for I2C Interrupts
void Shared_I2C_IRQHandler(I2C_Regs *i2c);

/* Diagnostics: how many transactions timed out and how many bus recoveries ran */
uint32_t I2C_GetTimeoutCount(void);
uint32_t I2C_GetRecoveryCount(void);

// Generic I2C write - i2c: I2C_0_INST or I2C_1_INST
I2C_Status I2C_WriteDevice(I2C_Regs *i2c, uint8_t dev_addr, uint8_t reg_addr, 
                           uint8_t *reg_data, uint8_t count);

// Generic I2C read - i2c: I2C_0_INST or I2C_1_INST
I2C_Status I2C_ReadDevice(I2C_Regs *i2c, uint8_t dev_addr, uint8_t reg_addr, 
                          uint8_t *reg_data, uint8_t count);

// Try to contact a single address - used by the scanner
bool I2C_TryAddress(I2C_Regs *i2c, uint8_t dev_addr);

// Scan the specified I2C bus and return list of responding addresses
uint8_t I2C_Scan(I2C_Regs *i2c, uint8_t *addr_list, uint8_t max_addrs);

// Backwards-compatible wrappers
void I2C_Write(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);
void I2C_Read(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

/********* Common Functions *********/

// Calculates CRC8
unsigned char CRC8(unsigned char *ptr, unsigned char len);

// Calculate checksum for RAM writes
unsigned char Checksum(unsigned char *ptr, unsigned char len);

#endif /* I2C_H_ */