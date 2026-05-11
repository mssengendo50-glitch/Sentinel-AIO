#ifndef SPI_CONTROLLER_H
#define SPI_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

#define SPI_PACKET_SIZE (512)

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