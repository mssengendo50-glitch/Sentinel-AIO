// Battery Gauge Library - MSPM0 + bq27Z746 (flash gauge)
// Clean unified version - no warnings, no duplicate files

#ifndef __GAUGE_H
#define __GAUGE_H

#include <stdbool.h>
#include <stdint.h>
#include "HAL/i2c.h"          // I2C_Regs + I2C_Status

// ================================================================
// Gauge I2C Address
// ================================================================
// ================================================================
// Common constants (from SLUA801)
// ================================================================
#define SOFT_RESET      0x0042
#define SET_CFGUPDATE   0x0013
#define CMD_DATA_CLASS  0x3E
#define CMD_BLOCK_DATA  0x40
#define CMD_CHECK_SUM   0x60
#define CMD_FLAGS       0x06
#define CFGUPD          0x0010
#define GAUGE_I2C_ADDR          0x55u

// ================================================================
// PUBLIC API
// ================================================================
int  gauge_read(I2C_Regs *i2c, uint8_t nRegister, uint8_t *pData, uint8_t nLength);
int  gauge_write(I2C_Regs *i2c, uint8_t nRegister, uint8_t *pData, uint8_t nLength);
void gauge_address(I2C_Regs *i2c, uint8_t nAddress);

//gauge_control: issues a sub command
//pHandle: handle to communications adapter
//nSubCmd: sub command number
//return value: result from sub command
unsigned int gauge_control(I2C_Regs *i2c, unsigned int nSubCmd);
//gauge_cmd_read: read data from standard command
//pHandle: handle to communications adapter
//nCmd: standard command
//return value: result from standard command
unsigned int gauge_cmd_read(I2C_Regs *i2c, uint8_t nCmd);
//gauge_cmd_write: write data to standard command
//pHandle: handle to communications adapter
//nCmd: standard command
//return value: number of bytes written to gauge
unsigned int gauge_cmd_write(I2C_Regs *i2c, uint8_t nCmd, unsigned int nData);
//gauge_cfg_update: enter configuration update mode for rom gauges
//pHandle: handle to communications adapter
//return value: true = success, false = failure
bool gauge_cfg_update(I2C_Regs *i2c);
//gauge_exit: exit configuration update mode for rom gauges
//pHandle: handle to communications adapter
//nSubCmd: sub command to exit configuration update mode
//return value: true = success, false = failure
bool gauge_exit(I2C_Regs *i2c, unsigned int nCmd);
//gauge_read_data_class: read a data class
//pHandle: handle to communications adapter
//nDataClass: data class number
//pData: buffer holding the whole data class (all blocks)
//nLength: length of data class (all blocks)
//return value: 0 = success
int gauge_read_data_class(I2C_Regs *i2c, uint8_t nDataClass, uint8_t *pData, uint8_t nLength);
//gauge_write_data_class: write a data class
//pHandle: handle to communications adapter
//nDataClass: data class number
//pData: buffer holding the whole data class (all blocks)
//nLength: length of data class (all blocks)
//return value: 0 = success
int gauge_write_data_class(I2C_Regs *i2c, uint8_t nDataClass, uint8_t *pData, uint8_t nLength);
//gauge_execute_fs: execute a flash stream file
//pHandle: handle to communications adapter
//pFS: zero-terminated buffer with flash stream file
//return value: success: pointer to end of flash stream file
//error: point of error in flashstream file
char *gauge_execute_fs(I2C_Regs *i2c, char *pFS);

#endif
