/*
 * Shared I2C Functions File for BQ7690x / BQ27Z7
 * Updated for Dual-Instance Support (I2C0 and I2C1)
 */

#include <stdio.h>
#include <stdbool.h>
#include "ti/devices/msp/peripherals/hw_i2c.h"
#include <ti/driverlib/dl_i2c.h>
#include "ti_msp_dl_config.h"
#include "i2c.h"

/* Configuration */

// Indicates status of I2C
enum I2cControllerStatus {
    I2C_STATUS_IDLE = 0,
    I2C_STATUS_TX_STARTED,
    I2C_STATUS_TX_INPROGRESS,
    I2C_STATUS_TX_COMPLETE,
    I2C_STATUS_RX_STARTED,
    I2C_STATUS_RX_INPROGRESS,
    I2C_STATUS_RX_COMPLETE,
    I2C_STATUS_ERROR,
} gI2cControllerStatus;

// Counters and Buffers
uint32_t gTxLen, gTxCount;
uint8_t gTxPacket[34];
uint8_t gRxPacket[34];
uint32_t gRxLen, gRxCount;

/********* I2C Master Driver Functions *********/

//initialize uart
void i2c_init(void){
    NVIC_ClearPendingIRQ(I2C_0_INST_INT_IRQN);
    // NVIC_ClearPendingIRQ(I2C_1_INST_INT_IRQN);
    NVIC_EnableIRQ(I2C_0_INST_INT_IRQN);
    // NVIC_EnableIRQ(I2C_1_INST_INT_IRQN);
}

// Shared Logic for I2C0 and I2C1 Interrupts
void Shared_I2C_IRQHandler(I2C_Regs *i2c) {
    switch (DL_I2C_getPendingInterrupt(i2c)) {
        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            gI2cControllerStatus = I2C_STATUS_RX_COMPLETE;
            break;
        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            DL_I2C_disableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            gI2cControllerStatus = I2C_STATUS_TX_COMPLETE;
            break;
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            gI2cControllerStatus = I2C_STATUS_RX_INPROGRESS;
            while (DL_I2C_isControllerRXFIFOEmpty(i2c) != true) {
                if (gRxCount < gRxLen) {
                    gRxPacket[gRxCount++] = DL_I2C_receiveControllerData(i2c);
                } else {
                    DL_I2C_receiveControllerData(i2c);
                }
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            gI2cControllerStatus = I2C_STATUS_TX_INPROGRESS;
            if (gTxCount < gTxLen) {
                gTxCount += DL_I2C_fillControllerTXFIFO(i2c, &gTxPacket[gTxCount], gTxLen - gTxCount);
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        case DL_I2C_IIDX_CONTROLLER_NACK:
            gI2cControllerStatus = I2C_STATUS_ERROR;
            break;
        default:
            break;
    }
}

I2C_Status I2C_WriteDevice(I2C_Regs *i2c, uint8_t dev_addr, uint8_t reg_addr, 
                           uint8_t *reg_data, uint8_t count) {
    gTxPacket[0] = reg_addr;
    for (int i = 0; i < count; i++) {
        gTxPacket[i+1] = reg_data[i];
    }

    DL_I2C_flushControllerTXFIFO(i2c);
    DL_I2C_fillControllerTXFIFO(i2c, &gTxPacket[0], count + 1);
    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);

    while (!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE));

    gI2cControllerStatus = I2C_STATUS_TX_STARTED; // Reset before transfer
    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, count + 1);

    while ((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) && 
           (gI2cControllerStatus != I2C_STATUS_ERROR));

    // Check result
    if (gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(i2c);
        return I2C_ERROR_NACK; 
    }

    while (DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    
    delay_cycles(1000);
    DL_I2C_flushControllerTXFIFO(i2c);
    
    return I2C_SUCCESS;
}

I2C_Status I2C_ReadDevice(I2C_Regs *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count){
    gRxLen   = count;
    gRxCount = 0;

    // Reset status
    gI2cControllerStatus = I2C_STATUS_RX_STARTED;

    // === Phase 1: Send register address (TX) ===
    gTxPacket[0] = reg_addr;
    DL_I2C_flushControllerTXFIFO(i2c);
    DL_I2C_fillControllerTXFIFO(i2c, gTxPacket, 1);
    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);

    while (!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    while ((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) &&
           (gI2cControllerStatus != I2C_STATUS_ERROR));

    if (gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(i2c);
        return I2C_ERROR_NACK;
    }

    while (DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    delay_cycles(1000);

    // === Phase 2: Read data (RX) ===
    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
                               DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, count);

    while ((gI2cControllerStatus != I2C_STATUS_RX_COMPLETE) &&
           (gI2cControllerStatus != I2C_STATUS_ERROR));

    if (gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerRXFIFO(i2c);
        return I2C_ERROR_NACK;
    }

    // Copy data
    for (uint8_t i = 0; i < count; i++) {
        reg_data[i] = gRxPacket[i];
    }

    // Cleanup
    DL_I2C_flushControllerTXFIFO(i2c);
    DL_I2C_flushControllerRXFIFO(i2c);

    return I2C_SUCCESS;
}
bool I2C_TryAddress(I2C_Regs *i2c, uint8_t dev_addr)
{
    uint8_t dummy = 0x00;
    
    // Reset the controller status before attempting transfer
    gI2cControllerStatus = I2C_STATUS_IDLE;
    
    DL_I2C_flushControllerTXFIFO(i2c);
    
    // Wait for idle state
    while (!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE));
    
    // Enable the TX interrupt to detect completion/error
    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    
    // Fill FIFO with dummy byte
    DL_I2C_fillControllerTXFIFO(i2c, &dummy, 1);
    
    // Start the transfer
    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    
    // Wait for completion or error
    while ((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) && 
           (gI2cControllerStatus != I2C_STATUS_ERROR));
    
    // Wait until bus is no longer busy
    while (DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    
    // Check if we got an error (NACK or arbitration lost)
    bool success = (gI2cControllerStatus == I2C_STATUS_TX_COMPLETE);
    
    // Clear any error conditions for next scan
    if (!success) {
        // Clear the error interrupt flags
        DL_I2C_clearInterruptStatus(i2c, DL_I2C_INTERRUPT_CONTROLLER_NACK | 
                                          DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
    }
    // Flush and reset
    DL_I2C_flushControllerTXFIFO(i2c);
    gI2cControllerStatus = I2C_STATUS_IDLE;
    
    delay_cycles(1000); 
    
    return success;
}

/********* Common Functions *********/

unsigned char CRC8(unsigned char *ptr, unsigned char len) {
    unsigned char i, crc = 0;
    while (len-- != 0) {
        for (i = 0x80; i != 0; i /= 2) {
            if ((crc & 0x80) != 0) { crc *= 2; crc ^= 0x107; }
            else crc *= 2;
            if ((*ptr & i) != 0) crc ^= 0x107;
        }
        ptr++;
    }
    return (crc);
}

unsigned char Checksum(unsigned char *ptr, unsigned char len) {
    unsigned char checksum = 0;
    for (uint8_t i = 0; i < len; i++) checksum += ptr[i];
    return (0xff & ~checksum);
}

/********* Hardware IRQ Handlers *********/

void I2C_0_INST_IRQHandler(void) {
    Shared_I2C_IRQHandler(I2C_0_INST);
}

void I2C_1_INST_IRQHandler(void) {
    Shared_I2C_IRQHandler(I2C_1_INST);
}