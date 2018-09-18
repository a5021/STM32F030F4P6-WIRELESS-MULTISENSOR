#ifndef __SPI_INCLUDED
#define __SPI_INCLUDED

#define SPI_SLEEP_WHILE_XFER()  NVIC_ClearPendingIRQ(SPI1_IRQn); __WFE()

__STATIC_INLINE void initSPI(void) {

  RCC->APB2ENR = RCC_APB2ENR_SPI1EN;  // enable clock for SPI1
  
  // SPI control register 2 (SPIx_CR2)
  // Address offset: 0x04
  // Reset value: 0x0700
  SPI1->CR2 = (
    0 * SPI_CR2_RXDMAEN  |            // Rx Buffer DMA Enable
    0 * SPI_CR2_TXDMAEN  |            // Tx Buffer DMA Enable
    0 * SPI_CR2_SSOE     |            // SS Output Enable
    0 * SPI_CR2_NSSP     |            // NSS pulse management Enable
    0 * SPI_CR2_FRF      |            // Frame Format Enable
    0 * SPI_CR2_ERRIE    |            // Error Interrupt Enable
    1 * SPI_CR2_RXNEIE   |            // RX buffer Not Empty Interrupt Enable
    0 * SPI_CR2_TXEIE    |            // Tx buffer Empty Interrupt Enable
      
    // DS[3:0] Data Size
    // These bits configure the data length for SPI transfers:
             // 0000: Not used
             // 0001: Not used
             // 0010: Not used
             // 0011: 4-bit
             // 0100: 5-bit
             // 0101: 6-bit
             // 0110: 7-bit
             // 0111: 8-bit
             // ^^^^^^^^^^^  use 8-bit data length
             // 1000: 9-bit
             // 1001: 10-bit
             // 1010: 11-bit
             // 1011: 12-bit
             // 1100: 13-bit
             // 1101: 14-bit
             // 1110: 15-bit
             // 1111: 16-bit      
      
    1 * SPI_CR2_DS_0     |            // Bit 0
    1 * SPI_CR2_DS_1     |            // Bit 1
    1 * SPI_CR2_DS_2     |            // Bit 2
    0 * SPI_CR2_DS_3     |            // Bit 3
    1 * SPI_CR2_FRXTH    |            // FIFO reception Threshold
    0 * SPI_CR2_LDMARX   |            // Last DMA transfer for reception
    0 * SPI_CR2_LDMATX                // Last DMA transfer for transmission
  ); 
  
  // SPI control register 1 (SPIx_CR1)
  // Address offset: 0x00
  // Reset value: 0x0000  
  SPI1->CR1 = (
    0 * SPI_CR1_CPHA     |            // Clock Phase
    0 * SPI_CR1_CPOL     |            // Clock Polarity
    1 * SPI_CR1_MSTR     |            // Master Selection

    // BR[2:0] bits (Baud Rate Control)
             // 000: fPCLK/2
             // 001: fPCLK/4 
             // 010: fPCLK/8
             // 011: fPCLK/16
             // 100: fPCLK/32
             // 101: fPCLK/64
             // 110: fPCLK/128
             // 111: fPCLK/256
      
    0 * SPI_CR1_BR_0     |            // Bit 0
    0 * SPI_CR1_BR_1     |            // Bit 1
    0 * SPI_CR1_BR_2     |            // Bit 2
    1 * SPI_CR1_SPE      |            // SPI Enable
    0 * SPI_CR1_LSBFIRST |            // Frame Format
    1 * SPI_CR1_SSI      |            // Internal slave select
    1 * SPI_CR1_SSM      |            // Software slave management
    0 * SPI_CR1_RXONLY   |            // Receive only
    0 * SPI_CR1_CRCL     |            // CRC Length
    0 * SPI_CR1_CRCNEXT  |            // Transmit CRC next
    0 * SPI_CR1_CRCEN    |            // Hardware CRC calculation enable
    0 * SPI_CR1_BIDIOE   |            // Output enable in bidirectional mode
    0 * SPI_CR1_BIDIMODE              // Bidirectional data mode enable
  );  
}                                    
                                    
#endif

