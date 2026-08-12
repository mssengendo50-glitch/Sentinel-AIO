/*
 * Shared I2C Functions File for BQ7690x / BQ27Z7
 * Updated for Dual-Instance Support (I2C0 and I2C1)
 *
 * HARDENED VERSION FOR SENTINEL AIO
 *  - every wait loop is bounded by a timeout (no more permanent main-loop hangs)
 *  - gI2cControllerStatus is volatile (it is written from the IRQ handler)
 *  - transactions are refused if the peripheral is powered down
 *  - a bus-recovery sequence (9 clocks + manual STOP) runs after a timeout
 *  - retains PIR_Interrupt_PauseForI2C / PIR_Interrupt_ResumeAfterI2C calls
 */

#include <stdio.h>
#include <stdbool.h>
#include "ti/devices/msp/peripherals/hw_i2c.h"
#include <ti/driverlib/dl_i2c.h>
#include "ti_msp_dl_config.h"
#include "i2c.h"

extern void PIR_Interrupt_PauseForI2C(void);
extern void PIR_Interrupt_ResumeAfterI2C(void);

/* ─────────────────────────────────────────────────────────────────────────────
 * Timeout configuration
 *
 * These are loop-iteration counts, not cycles. Each iteration is a peripheral
 * register read plus a decrement/compare, so roughly 8-12 CPU cycles at 32 MHz.
 * 200000 iterations therefore corresponds to somewhere around 50-75 ms, which
 * is far longer than any legitimate transaction on this bus (the longest is a
 * 34-byte block read at 100 kHz, about 3.5 ms) but short enough that a stuck
 * bus never stalls the state machine for a noticeable amount of time.
 * ───────────────────────────────────────────────────────────────────────────*/
#define I2C_TIMEOUT_LOOPS       (200000UL)

/* Half-period for the manual bus-recovery clock: ~5 us at 32 MHz => ~100 kHz */
#define I2C_RECOVERY_HALF_BIT   (160U)

/* Wait while `cond` is true. Sets `timed_out` if the loop budget is exhausted. */
#define I2C_WAIT_WHILE(cond, timed_out)                     \
    do {                                                    \
        uint32_t _to = I2C_TIMEOUT_LOOPS;                   \
        (timed_out) = false;                                \
        while (cond) {                                      \
            if (--_to == 0U) { (timed_out) = true; break; } \
        }                                                   \
    } while (0)

/* Configuration */

volatile enum I2cControllerStatus gI2cControllerStatus;

// Counters and Buffers
uint32_t gTxLen, gTxCount;
uint8_t gTxPacket[64];
uint8_t gRxPacket[64];
volatile uint32_t gRxLen, gRxCount;

/* Diagnostics — readable over the CLI to see whether the bus is misbehaving */
volatile uint32_t gI2cTimeoutCount   = 0;
volatile uint32_t gI2cRecoveryCount  = 0;

/********* I2C Master Driver Functions *********/

//initialize i2c
void i2c_init(void){
    NVIC_ClearPendingIRQ(I2C_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(I2C_1_INST_INT_IRQN);
    NVIC_EnableIRQ(I2C_0_INST_INT_IRQN);
    NVIC_EnableIRQ(I2C_1_INST_INT_IRQN);
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Power gating guard
 *
 * PWR_ExitMeasureProfile() calls DL_I2C_disablePower(). Reads of a powered-down
 * peripheral return 0, so a status-polling loop would never see its exit
 * condition. Every entry point checks this first.
 * ───────────────────────────────────────────────────────────────────────────*/
static bool I2C_IsPowered(I2C_Regs *i2c)
{
    if (i2c == I2C_0_INST) return g_i2c0_powered;
    if (i2c == I2C_1_INST) return g_i2c1_powered;
    return false;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Bus recovery
 *
 * Called after a timeout. A slave that was reset (or glitched) part way through
 * sending a byte can hold SDA low forever, which wedges the controller. Driving
 * up to 9 clock pulses lets it finish that byte, then a manual STOP releases the
 * bus. Finally the controller is reset and re-initialised.
 * ───────────────────────────────────────────────────────────────────────────*/
static void I2C_BusRecover(I2C_Regs *i2c)
{
    uint32_t sdaIomux, sclIomux, sdaFunc, sclFunc;
    GPIO_Regs *port;
    uint32_t sdaPin, sclPin;

    if (i2c == I2C_0_INST) {
        sdaIomux = GPIO_I2C_0_IOMUX_SDA;  sdaFunc = GPIO_I2C_0_IOMUX_SDA_FUNC;
        sclIomux = GPIO_I2C_0_IOMUX_SCL;  sclFunc = GPIO_I2C_0_IOMUX_SCL_FUNC;
        port     = GPIO_I2C_0_SDA_PORT;
        sdaPin   = GPIO_I2C_0_SDA_PIN;    sclPin  = GPIO_I2C_0_SCL_PIN;
    } else if (i2c == I2C_1_INST) {
        sdaIomux = GPIO_I2C_1_IOMUX_SDA;  sdaFunc = GPIO_I2C_1_IOMUX_SDA_FUNC;
        sclIomux = GPIO_I2C_1_IOMUX_SCL;  sclFunc = GPIO_I2C_1_IOMUX_SCL_FUNC;
        port     = GPIO_I2C_1_SDA_PORT;
        sdaPin   = GPIO_I2C_1_SDA_PIN;    sclPin  = GPIO_I2C_1_SCL_PIN;
    } else {
        return;
    }

    gI2cRecoveryCount++;

    /* Take manual control of the two pins. */
    DL_GPIO_initDigitalOutputFeatures(sclIomux,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_ENABLE);
    DL_GPIO_initDigitalInputFeatures(sdaIomux,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setPins(port, sclPin);          /* release SCL high */
    DL_GPIO_enableOutput(port, sclPin);
    delay_cycles(I2C_RECOVERY_HALF_BIT);

    /* Up to 9 clocks, stopping early once the slave lets SDA go high */
    for (uint8_t i = 0U; i < 9U; i++) {
        if ((DL_GPIO_readPins(port, sdaPin) & sdaPin) != 0U) {
            break;
        }
        DL_GPIO_clearPins(port, sclPin);    /* drive SCL low */
        delay_cycles(I2C_RECOVERY_HALF_BIT);
        DL_GPIO_setPins(port, sclPin);      /* release SCL high */
        delay_cycles(I2C_RECOVERY_HALF_BIT);
    }

    /* Manual STOP condition: SDA low -> high while SCL is high. */
    DL_GPIO_initDigitalOutputFeatures(sdaIomux,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_ENABLE);
    DL_GPIO_clearPins(port, sdaPin);        /* SDA low */
    DL_GPIO_enableOutput(port, sdaPin);
    delay_cycles(I2C_RECOVERY_HALF_BIT);
    DL_GPIO_setPins(port, sclPin);          /* SCL high */
    delay_cycles(I2C_RECOVERY_HALF_BIT);
    DL_GPIO_setPins(port, sdaPin);          /* SDA released high => STOP */
    delay_cycles(I2C_RECOVERY_HALF_BIT);

    /* Hand the pins back to the I2C peripheral */
    DL_GPIO_disableOutput(port, sdaPin);
    DL_GPIO_disableOutput(port, sclPin);
    DL_GPIO_initPeripheralInputFunctionFeatures(sdaIomux, sdaFunc,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(sclIomux, sclFunc,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(sdaIomux);
    DL_GPIO_enableHiZ(sclIomux);

    /* Reset and re-initialise the controller itself */
    DL_I2C_reset(i2c);
    DL_I2C_enablePower(i2c);
    delay_cycles(POWER_STARTUP_DELAY);
    if (i2c == I2C_0_INST) {
        SYSCFG_DL_I2C_0_init();
    } else {
        SYSCFG_DL_I2C_1_init();
    }
    i2c_init();

    gI2cControllerStatus = I2C_STATUS_IDLE;
}

/* Common exit path for a timed-out transaction */
static I2C_Status I2C_HandleTimeout(I2C_Regs *i2c)
{
    gI2cTimeoutCount++;
    I2C_BusRecover(i2c);
    return I2C_ERROR_TIMEOUT;
}

uint32_t I2C_GetTimeoutCount(void)  { return gI2cTimeoutCount;  }
uint32_t I2C_GetRecoveryCount(void) { return gI2cRecoveryCount; }

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
    bool timed_out;

    /* Never touch a powered-down peripheral */
    if (!I2C_IsPowered(i2c)) {
        return I2C_ERROR_TIMEOUT;
    }
    if (count > (uint8_t)(sizeof(gTxPacket) - 1U)) {
        return I2C_ERROR_NACK;
    }

    if (i2c == I2C_0_INST) {
        PIR_Interrupt_PauseForI2C();
    }

    gTxPacket[0] = reg_addr;
    for (int i = 0; i < count; i++) {
        gTxPacket[i+1] = reg_data[i];
    }

    DL_I2C_flushControllerTXFIFO(i2c);
    DL_I2C_fillControllerTXFIFO(i2c, &gTxPacket[0], count + 1);
    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);

    I2C_WAIT_WHILE(!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE), timed_out);
    if (timed_out) {
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return I2C_HandleTimeout(i2c);
    }

    gI2cControllerStatus = I2C_STATUS_TX_STARTED; // Reset before transfer
    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, count + 1);

    I2C_WAIT_WHILE((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) && 
                   (gI2cControllerStatus != I2C_STATUS_ERROR), timed_out);
    if (timed_out) {
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return I2C_HandleTimeout(i2c);
    }

    // Check result
    if (gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(i2c);
        if (i2c == I2C_0_INST) {
            PIR_Interrupt_ResumeAfterI2C();
        }
        return I2C_ERROR_NACK; 
    }

    I2C_WAIT_WHILE(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS, timed_out);
    if (timed_out) {
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return I2C_HandleTimeout(i2c);
    }
    
    delay_cycles(1000);
    DL_I2C_flushControllerTXFIFO(i2c);
    
    if (i2c == I2C_0_INST) {
        PIR_Interrupt_ResumeAfterI2C();
    }
    return I2C_SUCCESS;
}

I2C_Status I2C_ReadDevice(I2C_Regs *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count){
    bool timed_out;

    /* Never touch a powered-down peripheral */
    if (!I2C_IsPowered(i2c)) {
        return I2C_ERROR_TIMEOUT;
    }
    if (count > (uint8_t)sizeof(gRxPacket)) {
        return I2C_ERROR_NACK;
    }

    if (i2c == I2C_0_INST) {
        PIR_Interrupt_PauseForI2C();
    }

    gRxLen   = count;
    gRxCount = 0;

    // Reset status
    gI2cControllerStatus = I2C_STATUS_RX_STARTED;

    // === Phase 1: Send register address (TX) ===
    gTxPacket[0] = reg_addr;
    DL_I2C_flushControllerTXFIFO(i2c);
    DL_I2C_fillControllerTXFIFO(i2c, gTxPacket, 1);
    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);

    I2C_WAIT_WHILE(!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE), timed_out);
    if (timed_out) {
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return I2C_HandleTimeout(i2c);
    }

    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    I2C_WAIT_WHILE((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) &&
                   (gI2cControllerStatus != I2C_STATUS_ERROR), timed_out);
    if (timed_out) {
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return I2C_HandleTimeout(i2c);
    }

    if (gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(i2c);
        if (i2c == I2C_0_INST) {
            PIR_Interrupt_ResumeAfterI2C();
        }
        return I2C_ERROR_NACK;
    }

    I2C_WAIT_WHILE(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS, timed_out);
    if (timed_out) {
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return I2C_HandleTimeout(i2c);
    }

    delay_cycles(1000);

    // === Phase 2: Read data (RX) ===
    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
                               DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, count);

    I2C_WAIT_WHILE((gI2cControllerStatus != I2C_STATUS_RX_COMPLETE) &&
                   (gI2cControllerStatus != I2C_STATUS_ERROR), timed_out);
    if (timed_out) {
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return I2C_HandleTimeout(i2c);
    }

    if (gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerRXFIFO(i2c);
        if (i2c == I2C_0_INST) {
            PIR_Interrupt_ResumeAfterI2C();
        }
        return I2C_ERROR_NACK;
    }

    // Copy data
    for (uint8_t i = 0; i < count; i++) {
        reg_data[i] = gRxPacket[i];
    }

    // Cleanup
    DL_I2C_flushControllerTXFIFO(i2c);
    DL_I2C_flushControllerRXFIFO(i2c);

    if (i2c == I2C_0_INST) {
        PIR_Interrupt_ResumeAfterI2C();
    }
    return I2C_SUCCESS;
}

bool I2C_TryAddress(I2C_Regs *i2c, uint8_t dev_addr)
{
    uint8_t dummy = 0x00;
    bool timed_out;

    /* Never touch a powered-down peripheral */
    if (!I2C_IsPowered(i2c)) {
        return false;
    }

    if (i2c == I2C_0_INST) {
        PIR_Interrupt_PauseForI2C();
    }
    
    // Reset the controller status before attempting transfer
    gI2cControllerStatus = I2C_STATUS_IDLE;
    
    DL_I2C_flushControllerTXFIFO(i2c);
    
    // Wait for idle state
    I2C_WAIT_WHILE(!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE), timed_out);
    if (timed_out) {
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        (void)I2C_HandleTimeout(i2c);
        return false;
    }
    
    // Enable the TX interrupt to detect completion/error
    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    
    // Fill FIFO with dummy byte
    DL_I2C_fillControllerTXFIFO(i2c, &dummy, 1);
    
    // Start the transfer
    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    
    // Wait for completion or error
    I2C_WAIT_WHILE((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) && 
                   (gI2cControllerStatus != I2C_STATUS_ERROR), timed_out);
    if (timed_out) {
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        (void)I2C_HandleTimeout(i2c);
        return false;
    }
    
    // Wait until bus is no longer busy
    I2C_WAIT_WHILE(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS, timed_out);
    if (timed_out) {
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        (void)I2C_HandleTimeout(i2c);
        return false;
    }
    
    // Check if we got an error (NACK or arbitration lost)
    bool success = (gI2cControllerStatus == I2C_STATUS_TX_COMPLETE);
    
    // Clear any error conditions for next scan
    if (!success) {
        DL_I2C_clearInterruptStatus(i2c, DL_I2C_INTERRUPT_CONTROLLER_NACK |
                                          DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
    }
    // Flush and reset
    DL_I2C_flushControllerTXFIFO(i2c);
    gI2cControllerStatus = I2C_STATUS_IDLE;
    
    if (i2c == I2C_0_INST) {
        PIR_Interrupt_ResumeAfterI2C();
    }

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
