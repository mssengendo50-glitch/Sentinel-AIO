#include "ZDP323B.h"
#include "HAL/i2c.h"
#include "HAL/uart.h"

extern void PIR_interrupt(bool enable);
extern void PIR_Interrupt_PauseForI2C(void);
extern void PIR_Interrupt_ResumeAfterI2C(void);

ZDP323B_Device gPIR;

void ZDP323B_BuildConfigBytes(const ZDP323B_Config *cfg, uint8_t *out) {
    // out must be 7 bytes, will be sent B55 first

    // Bytes 0-2: all reserved, set to 0
    out[0] = 0x00;  // B55-B48
    out[1] = 0x00;  // B47-B40
    out[2] = 0x00;  // B39-B32

    // Byte 3: B31-B24
    // [B31:B29]=000 reserved
    // [B28:B26]=FILSEL[2:0]
    // [B25:B24]=FSTEP[1:0]
    out[3] = (uint8_t)(((cfg->filter_type & 0x07) << 2) |
                        (cfg->filter_step & 0x03));

    // Byte 4: B23-B16
    // [B23]=TRIGOM
    // [B22:B16]=DETLVL[7:1]  (upper 7 bits of threshold)
    out[4] = (uint8_t)(((cfg->trigger_en ? 1 : 0) << 7) |
                        ((cfg->threshold >> 1) & 0x7F));

    // Byte 5: B15-B8
    // [B15]=DETLVL[0]  (LSB of threshold)
    // [B14:B8]=reserved=0
    out[5] = (uint8_t)((cfg->threshold & 0x01) << 7);

    // Byte 6: all reserved
    out[6] = 0x00;  // B7-B0
}


bool I2C_TryAddress10(I2C_Regs *i2c, uint16_t dev_addr)
{
    if (dev_addr > 0x3FF) return false;
    bool timed_out;

    if (!g_i2c0_powered && i2c == I2C_0_INST) return false;
    if (!g_i2c1_powered && i2c == I2C_1_INST) return false;

    if (i2c == I2C_0_INST) {
        PIR_Interrupt_PauseForI2C();
    }

    uint8_t dummy = 0x00;

    gI2cControllerStatus = I2C_STATUS_IDLE;
    DL_I2C_flushControllerTXFIFO(i2c);

    // Switch to 10-bit addressing mode
    DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_10_BIT);

    I2C_WAIT_WHILE(!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE), timed_out);
    if (timed_out) {
        DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return false;
    }

    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);

    DL_I2C_fillControllerTXFIFO(i2c, &dummy, 1);

    // Pass full 10-bit address directly - hardware handles the framing
    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    I2C_WAIT_WHILE((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) &&
                   (gI2cControllerStatus != I2C_STATUS_ERROR), timed_out);
    if (timed_out) {
        DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return false;
    }

    I2C_WAIT_WHILE(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS, timed_out);

    bool success = (gI2cControllerStatus == I2C_STATUS_TX_COMPLETE);

    if (!success) {
        DL_I2C_clearInterruptStatus(i2c, DL_I2C_INTERRUPT_CONTROLLER_NACK |
                                           DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
    }

    DL_I2C_flushControllerTXFIFO(i2c);
    gI2cControllerStatus = I2C_STATUS_IDLE;

    // Restore 7-bit addressing mode for other devices on the bus
    DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);

    delay_cycles(1000);

    if (i2c == I2C_0_INST) {
        PIR_Interrupt_ResumeAfterI2C();
    }

    return success;
}

I2C_Status ZDP323B_WriteConfig(I2C_Regs *i2c, uint16_t dev_addr, uint8_t *config_bytes) {
    bool timed_out;

    if (!g_i2c0_powered && i2c == I2C_0_INST) return I2C_ERROR_TIMEOUT;
    if (!g_i2c1_powered && i2c == I2C_1_INST) return I2C_ERROR_TIMEOUT;
    
    if (i2c == I2C_0_INST) {
        PIR_Interrupt_PauseForI2C();
    }

    DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_10_BIT);
    DL_I2C_flushControllerTXFIFO(i2c);

    // Load all 7 bytes directly - no register address prefix
    for (int i = 0; i < 7; i++) {
        gTxPacket[i] = config_bytes[i];
    }

    DL_I2C_fillControllerTXFIFO(i2c, &gTxPacket[0], 7);
    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);

    I2C_WAIT_WHILE(!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE), timed_out);
    if (timed_out) {
        DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return I2C_ERROR_TIMEOUT;
    }

    gI2cControllerStatus = I2C_STATUS_TX_STARTED;
    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 7);

    I2C_WAIT_WHILE((gI2cControllerStatus != I2C_STATUS_TX_COMPLETE) &&
                   (gI2cControllerStatus != I2C_STATUS_ERROR), timed_out);
    if (timed_out) {
        DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return I2C_ERROR_TIMEOUT;
    }

    if (gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerTXFIFO(i2c);
        DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);
        if (i2c == I2C_0_INST) {
            PIR_Interrupt_ResumeAfterI2C();
        }
        return I2C_ERROR_NACK;
    }

    I2C_WAIT_WHILE(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS, timed_out);
    delay_cycles(1000);
    DL_I2C_flushControllerTXFIFO(i2c);

    DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);

    if (i2c == I2C_0_INST) {
        PIR_Interrupt_ResumeAfterI2C();
    }

    return I2C_SUCCESS;
}

I2C_Status ZDP323B_ReadPeakHold(I2C_Regs *i2c, uint16_t dev_addr, int16_t *peak_out) {
    bool timed_out;

    if (!g_i2c0_powered && i2c == I2C_0_INST) return I2C_ERROR_TIMEOUT;
    if (!g_i2c1_powered && i2c == I2C_1_INST) return I2C_ERROR_TIMEOUT;

    if (i2c == I2C_0_INST) {
        PIR_Interrupt_PauseForI2C();
    }

    gRxLen   = 2;
    gRxCount = 0;

    DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_10_BIT);

    gI2cControllerStatus = I2C_STATUS_RX_STARTED;

    DL_I2C_enableInterrupt(i2c, DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
                                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    I2C_WAIT_WHILE(!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE), timed_out);
    if (timed_out) {
        DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return I2C_ERROR_TIMEOUT;
    }

    DL_I2C_startControllerTransfer(i2c, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, 2);

    I2C_WAIT_WHILE((gI2cControllerStatus != I2C_STATUS_RX_COMPLETE) &&
                   (gI2cControllerStatus != I2C_STATUS_ERROR), timed_out);
    if (timed_out) {
        DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);
        if (i2c == I2C_0_INST) PIR_Interrupt_ResumeAfterI2C();
        return I2C_ERROR_TIMEOUT;
    }

    if (gI2cControllerStatus == I2C_STATUS_ERROR) {
        DL_I2C_flushControllerRXFIFO(i2c);
        DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);
        if (i2c == I2C_0_INST) {
            PIR_Interrupt_ResumeAfterI2C();
        }
        return I2C_ERROR_NACK;
    }

    // Extract 12-bit signed value
    uint16_t raw = ((uint16_t)(gRxPacket[0] & 0x0F) << 8) | gRxPacket[1];

    // Sign extend from bit 11
    if (raw & 0x0800) {
        *peak_out = (int16_t)(raw | 0xF000);
    } else {
        *peak_out = (int16_t)raw;
    }

    DL_I2C_flushControllerRXFIFO(i2c);
    DL_I2C_setControllerAddressingMode(i2c, DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);

    if (i2c == I2C_0_INST) {
        PIR_Interrupt_ResumeAfterI2C();
    }

    return I2C_SUCCESS;
}

// ─────────────────────────────────────────────
// Blocking Init
// ─────────────────────────────────────────────

I2C_Status ZDP323B_Init(I2C_Regs *i2c, uint16_t dev_addr,
                         ZDP323B_FilterStep step, ZDP323B_FilterType type,
                         uint8_t threshold) {

    gPIR.i2c              = i2c;
    gPIR.dev_addr         = dev_addr;
    gPIR.motion_detected  = false;
    gPIR.initialized      = false;

    // ── Phase 2: Write settle config ────────────
    ZDP323B_Config settle_cfg = {
        .threshold    = 0xFF,
        .trigger_en   = false,
        .filter_step  = step,
        .filter_type  = type,
    };

    uint8_t config_bytes[7];
    ZDP323B_BuildConfigBytes(&settle_cfg, config_bytes);

    I2C_Status status = ZDP323B_WriteConfig(i2c, dev_addr, config_bytes);
    if (status != I2C_SUCCESS) {
        uart_printf("[PIR] Init failed: could not write settle config\n");
        return status;
    }
    uart_printf("[PIR] Settle config written. Polling Peak Hold...\n");

    // ── Phase 3: Poll Peak Hold ──────────────────
    int16_t peak = 0;
    uint32_t poll_count = 0;

    while (1) {
        status = ZDP323B_ReadPeakHold(i2c, dev_addr, &peak);
        if (status != I2C_SUCCESS) {
            uart_printf("[PIR] Init failed: peak hold read error\n");
            return status;
        }
        int16_t abs_peak = (peak < 0) ? -peak : peak;

        if (poll_count % 100 == 0) {
            uart_printf("[PIR] Peak Hold = %d (waiting for |peak| < 127)\n", peak);
        }
        poll_count++;
        if (abs_peak < 0x7F) {
            uart_printf("[PIR] Signal settled. Peak Hold = %d\n", peak);
            break;
        }
    }

    // ── Phase 4: Wait T_STAB ─────────────────────
    uart_printf("[PIR] Waiting T_STAB (30s)...\n");
    for (uint8_t i = 30; i > 0; i--) {
        uart_printf("[PIR] T_STAB: %d seconds remaining\n", i);
        delay_cycles(32000000);
    }
    uart_printf("[PIR] T_STAB complete.\n");

    // ── Phase 5: Write armed config ──────────────
    gPIR.armed_cfg = (ZDP323B_Config){
        .threshold   = threshold,
        .trigger_en  = true,
        .filter_step = step,
        .filter_type = type,
    };

    ZDP323B_BuildConfigBytes(&gPIR.armed_cfg, config_bytes);
    status = ZDP323B_WriteConfig(i2c, dev_addr, config_bytes);
    if (status != I2C_SUCCESS) {
        uart_printf("[PIR] Init failed: could not write armed config\n");
        return status;
    }

    // ── Phase 6: Arm PIR interrupt ───────────────
    PIR_interrupt(true);

    gPIR.initialized = true;
    uart_printf("[PIR] Armed. Threshold = %d (%d ADC counts)\n",
                threshold, threshold * 8);

    return I2C_SUCCESS;
}

void ZDP323B_MotionISR(void) {
    gPIR.motion_detected = true;
}