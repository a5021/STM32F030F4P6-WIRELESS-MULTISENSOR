#include "stm32f0xx.h"
#include "bmp180.h"

/*=================================
    Conversion time
     (temperature)

    standard mode          4.5 ms
  ==================================
    Conversion time
      (pressure)

    ultra low power mode   4.5 ms
    standard mode          7.5 ms
    high resolution mode   13.5 ms
    ultra high res. mode   25.5 ms
    Advanced res. mode     76.5 ms
  ==================================*/

const BMP180_conv_param_t convParam[4] = {
  { BMP180_P0_MEASURE, 33  },  /* ~4.5 ms   (33*2+4) * 512 / 8000 = 4.48ms)    */
  { BMP180_P1_MEASURE, 56  },  /* ~7.5 ms   (56*2+4) * 512 / 8000 = 7.424ms)   */
  { BMP180_P2_MEASURE, 103 },  /* ~13.5 ms  (103*2+4) * 512 / 8000 = 13.44ms)  */
  { BMP180_P3_MEASURE, 197 }   /* ~25.5 ms  (197*2+4) * 512 / 8000 = 25.472ms) */
};

int32_t B5;

#define DMA_CH3_DISABLE() DMA1->IFCR = (    \
          DMA_IFCR_CGIF3    |               \
          DMA_IFCR_CTCIF3   |               \
          DMA_IFCR_CHTIF3   |               \
          DMA_IFCR_CTEIF3                   \
        );                                  \
        DMA1_Channel3->CCR = 0

bool bmp180_read_prom(uint8_t buf[]) {

  I2C_CR1_STOP();           // configure STOPF as the wakeup event
  I2C_START_WRITING(BMP180_I2C_ADDRESS, 1, BMP180_PROM_START_ADDR);
  I2C_SLEEP_UNTIL_STOP();   // sleep till STOP flag is set

  RCC->AHBENR = (   // configure the clock gating tree
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    | // enable clock for FLASH (in sleep mode)
#endif
    RCC_AHBENR_DMA1EN     | // enable clock for DMA1
    RCC_AHBENR_SRAMEN     | // enable clock for SRAM (in sleep mode)
    RCC_AHBENR_GPIOAEN      // enable clock for GPIOA
  );

  DMA1_Channel3->CPAR = (uint32_t) &I2C1->RXDR; // set DMA ch. source
  DMA1_Channel3->CMAR = (uint32_t) buf + 2;     // set DMA ch. destination
  DMA1_Channel3->CNDTR = BMP180_PROM_DATA_LEN;  // transfer length (bytes)

  DMA1_Channel3->CCR = (
    1 * DMA_CCR_EN        | // Channel enable
    1 * DMA_CCR_TCIE      | // Transfer complete interrupt enable
    0 * DMA_CCR_HTIE      | // Half Transfer interrupt enable
    1 * DMA_CCR_TEIE      | // Transfer error interrupt enable
    0 * DMA_CCR_DIR       | // Data transfer direction
    0 * DMA_CCR_CIRC      | // Circular mode
    0 * DMA_CCR_PINC      | // Peripheral increment mode
    1 * DMA_CCR_MINC      | // Memory increment mode
    // PSIZE[1:0] bits (Peripheral size)
    //    00: 8-bits
    //    01: 16-bits
    //    10: 32-bits
    //    11: Reserved
    0 * DMA_CCR_PSIZE_1   | // Bit 1
    0 * DMA_CCR_PSIZE_0   | // Bit 0
    // MSIZE[1:0] bits (Memory size)
    //    00: 8-bits
    //    01: 16-bits
    //    10: 32-bits
    //    11: Reserved
    0 * DMA_CCR_MSIZE_0   | // Bit 0
    0 * DMA_CCR_MSIZE_1   | // Bit 1
    // PL[1:0] bits(Channel Priority level)
    //    00: Low
    //    01: Medium
    //    10: High
    //    11: Very high
    0 * DMA_CCR_PL_0      | // Bit 0
    1 * DMA_CCR_PL_1      | // Bit 1
    0 * DMA_CCR_MEM2MEM     // Memory to memory mode
  );
  NVIC_ClearPendingIRQ(DMA1_Channel2_3_IRQn); // clear pending DMA interrupt

  I2C_CR1_DMA_RX();                // enable interrupt on "RX  not empty"
  I2C_START_READING(BMP180_I2C_ADDRESS, BMP180_PROM_DATA_LEN);  // request data

  NVIC_ClearPendingIRQ(I2C1_IRQn); // clear pending I2C interrupt

  uint32_t rcc_cfgr = RCC->CFGR;   // save current MCU speed value
  RUN_MCU_AT(HF_MHZ);              // run mcu at 1/2 mhz
  __WFE();                         // put MCU into the sleep mode
  RCC->CFGR = rcc_cfgr;            // restore MCU speed

  if ((I2C1->ISR & I2C_ERROR_MASK) != 0) {           // check for I2C errors
    uint32_t i2c_err = I2C1->ISR & I2C_ERROR_MASK;   // save status register
    I2C1->ICR = (                                    // reset error flas
      I2C_ICR_NACKCF |
      I2C_ICR_BERRCF |
      I2C_ICR_ARLOCF
    );
    DMA_CH3_DISABLE();                               // disable DMA ch. 3

    RCC->AHBENR = (     // configure clock gating
#ifndef SWD_DISABLED
      RCC_AHBENR_FLITFEN    |  // enable clock for FLASH
      RCC_AHBENR_SRAMEN     |  // enable clock for SRAM
#endif
      RCC_AHBENR_GPIOAEN       // enable clock for GPIOA
    );

    ABANDON_ROUTINE(i2c_err >> 4);  // save error value and exit
  }

  if ((DMA1->ISR & DMA_ISR_TCIF3) != DMA_ISR_TCIF3) { // check for DMA errors
    uint32_t dma_err = DMA1->ISR;                     // save status register
    DMA_CH3_DISABLE();                                // disable DMA ch. 3

    RCC->AHBENR = (     // configure clock gating
#ifndef SWD_DISABLED
      RCC_AHBENR_FLITFEN    | // enable clock for FLASH
      RCC_AHBENR_SRAMEN     | // enable clock for SRAM
#endif
      RCC_AHBENR_GPIOAEN      // enable clock for GPIOA
    );

    if ((dma_err & DMA_ISR_TEIF3) == DMA_ISR_TEIF3) {  // check for transfer err
      ABANDON_ROUTINE(ERROR_I2C_DMA_TRANSFER_FAIL);    // reporot error and exit
    } else {
      ABANDON_ROUTINE(ERROR_I2C_UNKNOWN);              // reporot error and exit
    }
  }

  I2C1->ICR = I2C_ICR_STOPCF;                          // reset I2C STOP flag
  DMA_CH3_DISABLE();                                   // disable DMA ch. 3

  RCC->AHBENR = (      // configure clock gating
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    | // enable clock for FLASH
    RCC_AHBENR_SRAMEN     | // enable clock for SRAM
#endif
    RCC_AHBENR_GPIOAEN      // enable clock for GPIOA
  );

  // reverse byte order
  uint32_t * p32 = (uint32_t *) buf;    // get buffer head
  for (int i = 0; i < 6; i++) {         // for every 4-byte word
    *p32 = __REV16(*p32);               // swap two pairs of bytes
    p32++;                              // set pointer to the next buffer value
  }

  return true;
}

bool bmp180_get_prom(void) {

  BMP180_Calibration_TypeDef prom_buf;  // define the calibration data buffer

  if (!bmp180_read_prom((uint8_t*)&prom_buf)) { // read calibr. data from sensor
    return false;                               // break if error
  }

  prom_buf.PAD = 0xAAAA;                              // padding head of buffer
  prom_buf.CRC32 = crc32_prom((uint32_t*)&prom_buf);  // get checksum

  if (prom_buf.CRC32 != CAL_DATA->CRC32) {            // verify checksum
    // store calibration data into the FLASH
    if (!write2flash((uint16_t *) &prom_buf, sizeof(prom_buf) / 2)) {
      // in case of flash writing error
      nvStatus &= ~NV_BMP180_CAL_DATA_AVAILABLE; // reset flag
      return false;                              // return error
    }
  }

  nvStatus |= NV_BMP180_CAL_DATA_AVAILABLE; // report the calibr. data available
  return true;                              // return success
}
