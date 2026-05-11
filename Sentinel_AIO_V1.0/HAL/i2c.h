#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

/* * BQ7690x / BQ27Z7 I2C Functions Header File
 * Updated for Dual-Instance Support (I2C0 and I2C1)
 */

/********* I2C Master Driver Functions *********/

// In i2c.h:
typedef enum {
    I2C_SUCCESS = 0,
    I2C_ERROR_NACK,
    I2C_ERROR_TIMEOUT,
    I2C_ERROR_ARB_LOST
} I2C_Status;

void i2c_init(void);
// Shared logic for I2C Interrupts
void Shared_I2C_IRQHandler(I2C_Regs *i2c);

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

// Backwards-compatible wrappers (Note: These will likely need a default bus defined in your .c)
void I2C_Write(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);
void I2C_Read(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

/********* Common Functions *********/

// Calculates CRC8
unsigned char CRC8(unsigned char *ptr, unsigned char len);

// Calculate checksum for RAM writes
unsigned char Checksum(unsigned char *ptr, unsigned char len);

#endif /* I2C_H_ */