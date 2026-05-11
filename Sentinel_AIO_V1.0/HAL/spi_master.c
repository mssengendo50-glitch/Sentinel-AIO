#include "spi_master.h"
#include <string.h>

SPI_Controller_Handle stm32Spi; 

uint8_t gSPI_TxPacket[SPI_PACKET_SIZE];
uint8_t gSPI_RxPacket[SPI_PACKET_SIZE];
void spi_init(void) {
    NVIC_ClearPendingIRQ(SPI_1_INST_INT_IRQN);
    NVIC_EnableIRQ(SPI_1_INST_INT_IRQN);
    SPI_Controller_Init(&stm32Spi, SPI_1_INST,  DMA_CH0_CHAN_ID, DMA_CH1_CHAN_ID, gSPI_TxPacket, gSPI_RxPacket, SPI_PACKET_SIZE);
}

void SPI_Controller_Init(SPI_Controller_Handle *handle, SPI_Regs *spi, uint8_t txCh, uint8_t rxCh, uint8_t *txBuf, uint8_t *rxBuf, uint16_t len) 
{
    handle->spiInst = spi;
    handle->dmaInst = DMA;
    handle->txDmaCh = txCh;
    handle->rxDmaCh = rxCh;
    handle->txBuf   = txBuf;
    handle->rxBuf   = rxBuf;
    handle->size    = len;
    
    handle->txDone  = false;
    handle->rxDone  = false;
    handle->spiTransmitted = false;
}

void SPI_Controller_Arm(SPI_Controller_Handle *handle) {
    // 1. Reset Logic Flags
    handle->txDone  = false;
    handle->rxDone  = false;
    handle->spiTransmitted = false;

    // 2. Clear RX buffer
    memset(handle->rxBuf, 0, handle->size);

    // 3. Configure DMA TX (Memory -> SPI)
    DL_DMA_setSrcAddr(handle->dmaInst, handle->txDmaCh, (uint32_t)handle->txBuf);
    DL_DMA_setDestAddr(handle->dmaInst, handle->txDmaCh, (uint32_t)(&handle->spiInst->TXDATA));
    DL_DMA_setTransferSize(handle->dmaInst, handle->txDmaCh, handle->size);

    // 4. Configure DMA RX (SPI -> Memory)
    DL_DMA_setSrcAddr(handle->dmaInst, handle->rxDmaCh, (uint32_t)(&handle->spiInst->RXDATA));
    DL_DMA_setDestAddr(handle->dmaInst, handle->rxDmaCh, (uint32_t)handle->rxBuf);
    DL_DMA_setTransferSize(handle->dmaInst, handle->rxDmaCh, handle->size);

    // 5. Enable Channels - RX first, then TX to start clock
    DL_DMA_enableChannel(handle->dmaInst, handle->rxDmaCh);
    DL_DMA_enableChannel(handle->dmaInst, handle->txDmaCh);
}


void SPI_1_INST_IRQHandler(void) {
    switch (DL_SPI_getPendingInterrupt(SPI_1_INST)) {
        case DL_SPI_IIDX_DMA_DONE_TX:
            stm32Spi.txDone = true;
            break;
        case DL_SPI_IIDX_TX_EMPTY:
            stm32Spi.spiTransmitted = true;
            break;
        case DL_SPI_IIDX_DMA_DONE_RX:
            stm32Spi.rxDone = true;
            break;
        default: break;
    }
}

