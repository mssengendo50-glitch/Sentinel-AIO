#ifndef SPI_CONTROLLER_H
#define SPI_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "spi_protocol.h"

/*
 * THE TRANSFER LENGTH IS PART OF THE PROTOCOL. IT MUST EQUAL THE STM32's.
 *
 * This was 600 against an SM_SpiPacket_t of 512, and that 88-byte overrun is
 * what stopped the STM32 ever reading the clock from this supervisor.
 *
 * We are the SPI controller: we generate every edge. The STM32 is the slave and
 * arms a DMA of exactly MSPMO_BUFFER_SIZE (512) bytes per transfer. Clocking
 * 600 means its DMA completes 88 bytes before ours does - and the STM32 does not
 * wait around afterwards. On completion it raises MSP_SPI_TX_RX_CPLT, moves to
 * its Rx_response state, arms a fresh 512-byte DMA and toggles IO2 again. All of
 * that happens while we are still clocking the tail of the previous transfer.
 *
 * The result, from this side: the second IO2 edge arrives BEFORE rxDone is set,
 * so the dispatch block has nothing to do, the IO2 block re-arms, and
 * SPI_Controller_Arm()'s memset destroys the request we were in the middle of
 * receiving. The log shows two "sending offer" lines and then a frame of zeros,
 * which is the STM32's response-leg dummy buffer rather than its request.
 *
 * It is also why the shutdown always worked while the time request never did:
 * MSG_SHUTDOWN is a one-way Transmit_DMA whose meaningful bytes are the first
 * four, and those land correctly at the head of the frame. Anything needing a
 * REPLY needs the two transfers to stay framed, and they cannot be.
 *
 * Insect_Intel_V1.0 has always had 512 here, which is the entire reason the
 * identical state machine works on that product.
 *
 * Derived rather than written out, so it cannot drift from the packet again,
 * and asserted so a change to either one fails the build instead of the board.
 */
#define SPI_PACKET_SIZE ((uint16_t)sizeof(SM_SpiPacket_t))

/* Guarded because nothing else in this project uses _Static_assert and the
   build does not pin a C standard, so it is not worth risking a build break to
   find out. The typedef form is the C89-compatible equivalent and fails just as
   loudly - a negative array size. */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(SPI_PACKET_SIZE == 512U,
    "SPI_PACKET_SIZE must match MSPMO_BUFFER_SIZE (512) in the STM32's shared.h - "
    "the STM32 arms a DMA of exactly that many bytes per transfer");
#else
typedef char spi_packet_size_must_match_stm32_MSPMO_BUFFER_SIZE
    [(SPI_PACKET_SIZE == 512U) ? 1 : -1];
#endif

/**
 * @brief SPI Controller Handle
 */
typedef struct {
    SPI_Regs  *spiInst;      // Pointer to SPI instance
    DMA_Regs  *dmaInst;      // Pointer to DMA controller
    uint8_t    txDmaCh;      // DMA Channel ID for TX
    uint8_t    rxDmaCh;      // DMA Channel ID for RX
    
    uint8_t   *txBuf;        // Source data
    uint8_t   *rxBuf;        // Destination for incoming data
    uint16_t   size;
    
    /* Status Flags */
    volatile bool txDone;
    volatile bool rxDone;
    volatile bool spiTransmitted;
} SPI_Controller_Handle;

/**
 * @brief Initializes the controller handle.
 */
void SPI_Controller_Init(SPI_Controller_Handle *handle, 
                         SPI_Regs *spi, 
                         uint8_t txCh, uint8_t rxCh,
                         uint8_t *txBuf, uint8_t *rxBuf, 
                         uint16_t len);

/**
 * @brief Re-arms DMA and triggers a new transfer.
 */
void SPI_Controller_Arm(SPI_Controller_Handle *handle);
extern SPI_Controller_Handle stm32Spi;
extern uint8_t gSPI_TxPacket[SPI_PACKET_SIZE];
extern uint8_t gSPI_RxPacket[SPI_PACKET_SIZE];

void spi_init();


#endif /* SPI_CONTROLLER_H */