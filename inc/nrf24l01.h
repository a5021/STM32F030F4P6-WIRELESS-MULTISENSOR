#ifndef __NRF24L01_INCLUDED
#define __NRF24L01_INCLUDED

#define NRF24_PAYLOAD_SIZE         10

/* Memory Map */
#define CONFIG                     0x00
#define EN_AA                      0x01
#define EN_RXADDR                  0x02
#define SETUP_AW                   0x03
#define SETUP_RETR                 0x04
#define RF_CH                      0x05
#define RF_SETUP                   0x06
#define STATUS                     0x07
#define OBSERVE_TX                 0x08
#define CD                         0x09
#define RX_ADDR_P0                 0x0A
#define RX_ADDR_P1                 0x0B
#define RX_ADDR_P2                 0x0C
#define RX_ADDR_P3                 0x0D
#define RX_ADDR_P4                 0x0E
#define RX_ADDR_P5                 0x0F
#define TX_ADDR                    0x10
#define RX_PW_P0                   0x11
#define RX_PW_P1                   0x12
#define RX_PW_P2                   0x13
#define RX_PW_P3                   0x14
#define RX_PW_P4                   0x15
#define RX_PW_P5                   0x16
#define FIFO_STATUS                0x17
#define DYNPD	                     0x1C
#define FEATURE	                   0x1D

/* Bit Mnemonics */
#define MASK_RX_DR                 6
#define MASK_TX_DS                 5
#define MASK_MAX_RT                4
#define EN_CRC                     3
#define CRCO                       2
#define PWR_UP                     1
#define PRIM_RX                    0
#define ENAA_P5                    5
#define ENAA_P4                    4
#define ENAA_P3                    3
#define ENAA_P2                    2
#define ENAA_P1                    1
#define ENAA_P0                    0
#define ERX_P5                     5
#define ERX_P4                     4
#define ERX_P3                     3
#define ERX_P2                     2
#define ERX_P1                     1
#define ERX_P0                     0
#define AW                         0
#define ARD                        4
#define ARC                        0
#define PLL_LOCK                   4
#define RF_DR                      3
#define RF_PWR                     6
#define RX_DR                      6
#define TX_DS                      5
#define MAX_RT                     4
#define RX_P_NO                    1
#define TX_FULL                    0
#define PLOS_CNT                   4
#define ARC_CNT                    0
#define TX_REUSE                   6
#define FIFO_FULL                  5
#define TX_EMPTY                   4
#define RX_FULL                    1
#define RX_EMPTY                   0
#define DPL_P5	                   5
#define DPL_P4	                   4
#define DPL_P3	                   3
#define DPL_P2	                   2
#define DPL_P1	                   1
#define DPL_P0	                   0
#define EN_DPL	                   2
#define EN_ACK_PAY                 1
#define EN_DYN_ACK                 0

/* Instruction Mnemonics */
#define R_REGISTER                 0x00
#define W_REGISTER                 0x20
#define REGISTER_MASK              0x1F
#define ACTIVATE                   0x50
#define R_RX_PL_WID                0x60
#define R_RX_PAYLOAD               0x61
#define W_TX_PAYLOAD               0xA0
#define W_ACK_PAYLOAD              0xA8
#define FLUSH_TX                   0xE1
#define FLUSH_RX                   0xE2
#define REUSE_TX_PL                0xE3
#define NOP                        0xFF

#define W_TX_PAYLOAD_NOACK         0xB0

/* Non-P omissions */
#define LNA_HCURR                  0

/* P model memory Map */
#define RPD                        0x09

/* P model bit Mnemonics */
#define RF_DR_LOW                  5
#define RF_DR_HIGH                 3
#define RF_PWR_LOW                 1
#define RF_PWR_HIGH                2

// 00: CONFIG register bits from 0 to 6, bit 7 must be set to 0
// ============================================================
#define NRF24_CONFIG_PRIM_RX       (1 << PRIM_RX)
#define NRF24_CONFIG_PWR_UP        (1 << PWR_UP)
#define NRF24_CONFIG_CRCO          (1 << CRCO)
#define NRF24_CONFIG_EN_CRC        (1 << EN_CRC)
#define NRF24_CONFIG_MAX_RT        (1 << MAX_RT)
#define NRF24_CONFIG_TX_DS         (1 << TX_DS)
#define NRF24_CONFIG_RX_DR         (1 << RX_DR)

// 01: EN_AA register -- Enable ‘Auto Acknowledgment’ Function (bits from 0 to 5)
// ==============================================================================
#define NRF24_ENAA_P0              (1 << ENAA_P0)
#define NRF24_ENAA_P1              (1 << ENAA_P1)
#define NRF24_ENAA_P2              (1 << ENAA_P2)
#define NRF24_ENAA_P3              (1 << ENAA_P3)
#define NRF24_ENAA_P4              (1 << ENAA_P4)
#define NRF24_ENAA_P5              (1 << ENAA_P5)

// 02: EN_RXADDR -- Enable RX Pipe (bits from 0 to 5)
// ==================================================
#define NRF24_EN_RXADDR_ERX_P0     (1 << ERX_P0)
#define NRF24_EN_RXADDR_ERX_P1     (1 << ERX_P1)
#define NRF24_EN_RXADDR_ERX_P2     (1 << ERX_P2)
#define NRF24_EN_RXADDR_ERX_P3     (1 << ERX_P3)
#define NRF24_EN_RXADDR_ERX_P4     (1 << ERX_P4)
#define NRF24_EN_RXADDR_ERX_P5     (1 << ERX_P5)

// 03: SETUP_AW -- Setup of Address Widths (bits from 0 to 1)
// ==========================================================
//    RX/TX Address field width
//            '00' - Illegal
//            '01' - 3 bytes
//            '10' - 4 bytes
//            '11' – 5 bytes
// LSByte is used if address width is below 5 bytes
//
#define NRF24_SETUP_AW_0           (1 << 0)
#define NRF24_SETUP_AW_1           (1 << 1)
   
#define  NRF24_ADDR_WIDTH_3        (NRF24_SETUP_AW_0)
#define  NRF24_ADDR_WIDTH_4        (NRF24_SETUP_AW_1)
#define  NRF24_ADDR_WIDTH_5        (NRF24_SETUP_AW_0 | NRF24_SETUP_AW_1)

// 04: SETUP_RETR -- Setup of Automatic Retransmission (bits from 0 to 7)
// ======================================================================
// --
//     Auto Retransmit Count
// ‘0000’ –Re-Transmit disabled
// ‘0001’ – Up to 1 Re-Transmit on fail of AA
// ……
// ‘1111’ – Up to 15 Re-Transmit on fail of AA
//
#define NRF24_SETUP_RETR_ARC_0     (1 << 0)
#define NRF24_SETUP_RETR_ARC_1     (1 << 1)
#define NRF24_SETUP_RETR_ARC_2     (1 << 2)
#define NRF24_SETUP_RETR_ARC_3     (1 << 3)
// 
//  Auto Retransmit Delay
//  ‘0000’ – Wait 250µS
//  ‘0001’ – Wait 500µS
//  ‘0010’ – Wait 750µS
//  ……..
//  ‘1111’ – Wait 4000µS
//  (Delay defined from end of transmission to start of
//  next transmission)
//
#define NRF24_SETUP_RETR_ARD_0     (1 << 4)
#define NRF24_SETUP_RETR_ARD_1     (1 << 5)
#define NRF24_SETUP_RETR_ARD_2     (1 << 6)
#define NRF24_SETUP_RETR_ARD_3     (1 << 7)

// 05: RF_CH -- Radio Frequency Channel Selection (bits from 0 to 6)
// =================================================================
//
// 06: RF_SETUP -- RF Setup Register (bits from 0 to 6)
// ====================================================
//  Set RF output power in TX mode
//  '00' – -18dBm
//  '01' – -12dBm
//  '10' – -6dBm
//  '11' – 0dBm
//
#define NRF24_RF_SETUP_RF_PWR_0    (1 << 0)
#define NRF24_RF_SETUP_RF_PWR_1    (1 << 1)
//  Select between the high speed data rates. This bit
//  is don’t care if RF_DR_LOW is set.
//  Encoding:
//  [RF_DR_LOW, RF_DR_HIGH]:
//  ‘00’ – 1Mbps
//  ‘01’ – 2Mbps
//  ‘10’ – 250kbps
//  ‘11’ – Reserved
//
#define NRF24_RF_SETUP_RF_DR_HIGH  (1 << RF_DR_HIGH)
#define NRF24_RF_SETUP_PLL_LOCK    (1 << PLL_LOCK)
#define NRF24_RF_SETUP_RF_DR_LOW   (1 << RF_DR_LOW)
// bit 6 -- reserved 
#define NRF24_RF_SETUP_CONT_WAVE   (1 << CONT_WAVE)

// 07: STATUS -- Status Register (In parallel to the SPI command
//               word applied on the MOSI pin, the STATUS register
//               is shifted serially out on the MISO pin)   (bits from 0 to 6)
// ===============================================================
//  TX FIFO full flag.
//  1: TX FIFO full.
//  0: Available locations in TX FIFO.
//
#define NRF24_STATUS_TX_FULL       (1 << TX_FULL)   
//
//  Data pipe number for the payload available for
//  reading from RX_FIFO
//  000-101: Data Pipe Number
//  110: Not Used
//  111: RX FIFO Empty
//
#define NRF24_STATUS_RX_P_NO_0     (1 << 1)
#define NRF24_STATUS_RX_P_NO_1     (1 << 2)
#define NRF24_STATUS_RX_P_NO_2     (1 << 3)
#define NRF24_STATUS_MAX_RT        (1 << MAX_RT)
#define NRF24_STATUS_TX_DS         (1 << TX_DS)
#define NRF24_STATUS_RX_DR         (1 << RX_DR)

// 08: OBSERVE_TX -- Transmit observe register
// ===========================================
//  Count retransmitted packets. The counter is reset
//  when transmission of a new packet starts.
//
#define NRF24_OBSERVE_TX_ARC_CNT_0 (1 << 0) 
#define NRF24_OBSERVE_TX_ARC_CNT_1 (1 << 1)
#define NRF24_OBSERVE_TX_ARC_CNT_2 (1 << 2)
#define NRF24_OBSERVE_TX_ARC_CNT_3 (1 << 3)
//
//  Count lost packets. The counter is overflow protected to 15, 
//  and discontinues at max until reset.
//  The counter is reset by writing to RF_CH.
//
#define NRF24_OBSERVE_TX_PLOS_CNT_0 (1 << 4)
#define NRF24_OBSERVE_TX_PLOS_CNT_1 (1 << 5)
#define NRF24_OBSERVE_TX_PLOS_CNT_2 (1 << 6)
#define NRF24_OBSERVE_TX_PLOS_CNT_3 (1 << 7)

// 09: RPD -- Received Power Detector. Only bit 0.
// ===============================================
#define NRF24_RPD                  (1 << RPD)

// 0A: RX_ADDR_P0 -- Receive address data pipe 0. 24 - 40 bits, depending on SETUP_AW register
// ===========================================================================================

// 0B: RX_ADDR_P1 -- Receive address data pipe 1. 24 - 40 bits, depending on SETUP_AW register
// ===========================================================================================
// ...
// ...
// 0F: RX_ADDR_P5 -- Receive address data pipe 1. 24 - 40 bits, depending on SETUP_AW register
// ===========================================================================================

// 10: TX_ADDR -- Transmit address. Used for a PTX device only.
// ============================================================

// 11: RX_PW_P0 --  Number of bytes in RX payload in data pipe 0 (1 to 32 bytes)
// =============================================================================

// 11: RX_PW_P1 --  Number of bytes in RX payload in data pipe 1 (1 to 32 bytes)
// =============================================================================
// ...
// ...
// 16: RX_PW_P5 --  Number of bytes in RX payload in data pipe 1 (1 to 32 bytes)
// =============================================================================

// 17: FIFO_STATUS -- FIFO Status Register (bits 0..1 and 4..6)
// ============================================================
#define NRF24_FIFO_STATUS_RX_EMPTY (1 << RX_EMPTY)
#define NRF24_FIFO_STATUS_RX_FULL  (1 << RX_FULL)
// bits 2:3 are reserved 
#define NRF24_FIFO_STATUS_TX_EMPTY (1 << TX_EMPTY)
#define NRF24_FIFO_STATUS_TX_FULL  (1 << TX_FULL)
#define NRF24_FIFO_STATUS_TX_REUSE (1 << TX_REUSE)

// 1C: DYNPD -- Dynamic payload register (bits from 0 to 6)
// ========================================================
#define NRF24_DYNPD_DPL_P0         (1 << DPL_P0)
#define NRF24_DYNPD_DPL_P1         (1 << DPL_P1)
#define NRF24_DYNPD_DPL_P2         (1 << DPL_P2)
#define NRF24_DYNPD_DPL_P3         (1 << DPL_P3)
#define NRF24_DYNPD_DPL_P4         (1 << DPL_P4)
#define NRF24_DYNPD_DPL_P5         (1 << DPL_P5)
#define NRF24_DYNPD_DPL_P6         (1 << DPL_P6)

// 1D: FEATURE -- Feature Register (bits from 0 to 2)
// ==================================================
#define NRF24_FEATURE_EN_DYN_ACK   (1 << EN_DYN_ACK)
#define NRF24_FEATURE_EN_ACK_PAY   (1 << EN_ACK_PAY)
#define NRF24_FEATURE_EN_DPL       (1 << EN_DPL)

#define NRF24_EN_PAYLOAD_NOACK     NRF24_FEATURE_EN_ACK_PAY
#define NRF24_EN_DYN_PAYLOAD       NRF24_FEATURE_EN_DPL

#define CSN_LOW()                  GPIOA->BSRR = GPIO_BSRR_BR_4
#define CSN_HIGH()                 GPIOA->BSRR = GPIO_BSRR_BS_4
#define CE_LOW()                   GPIOA->BSRR = GPIO_BSRR_BR_3
#define CE_HIGH()                  GPIOA->BSRR = GPIO_BSRR_BS_3

#define  NRF24_AUTO_ACK_DISABLE    ((uint8_t)(0))

#define  NRF24_DATA_RATE_250K      (1 << RF_DR_LOW)
#define  NRF24_DATA_RATE_1M        (0)
#define  NRF24_DATA_RATE_2M        (1 << RF_DR_HIGH)
  
#define  NRF24_OUTPUT_POWER_18DBM  (0)
#define  NRF24_OUTPUT_POWER_12DBM  (1 << RF_PWR_LOW)
#define  NRF24_OUTPUT_POWER_6DBM   (1 << RF_PWR_HIGH)
#define  NRF24_OUTPUT_POWER_0DBM   ((1 << RF_PWR_LOW) | (1 << RF_PWR_HIGH))

#define  NRF24_TX_POWER_LOWEST     NRF24_OUTPUT_POWER_18DBM
#define  NRF24_TX_POWER_LOW        NRF24_OUTPUT_POWER_12DBM
#define  NRF24_TX_POWER_NORMAL     NRF24_OUTPUT_POWER_6DBM
#define  NRF24_TX_POWER_HIGH       NRF24_OUTPUT_POWER_0DBM

#define NRF24_SET_ADDR_WIDTH(ADDR_WIDTH) \
          NRF_WRITE_REG(SETUP_AW, (ADDR_WIDTH))
            
#define NRF24_SET_AUTO_ACK(AA_STATE) \
          NRF_WRITE_REG(EN_AA, (AA_STATE))
            
#define NRF24_SET_RADIO_CHANNEL(CH) \
          NRF_WRITE_REG(RF_CH, (CH))
            
#define NRF24_SETUP_RADIO(TX_POWER, DATA_RATE) \
          NRF_WRITE_REG(RF_SETUP, ((DATA_RATE) | (TX_POWER)))

#define NRF24_SET_DATA_RATE(DATA_RATE) \
          NRF_WRITE_REG(RF_SETUP, DATA_RATE)
           
#define NRF24_SET_DATA_RATE_250K() \
          nrf_write_register(RF_SETUP, NRF24_DATA_RATE_250K)
        
#define NRF24_SET_DATA_RATE_1M() \
          nrf_write_register(RF_SETUP, NRF24_DATA_RATE_1M)
        
#define NRF24_SET_DATA_RATE_2M() \
          nrf_write_register(RF_SETUP, NRF24_DATA_RATE_2M)
        
/*

Page 30 of nRF24L01+ Product Specification:

In order to enable DPL the EN_DPL bit in the FEATURE register must be enabled. 
In RX mode the DYNPD register must be set. A PTX that transmits to a PRX with 
DPL enabled must have the DPL_P0 bit in DYNPD set.

*/

#define NRF24_SET_FEATURE(FT) \
          NRF_WRITE_REG(FEATURE, FT)
            
#define NRF24_SET_DPL_PIPE(DPL_PIPE) \
          NRF_WRITE_REG(DYNPD, (DPL_PIPE))

#define NRF_WRITE_CMD(CMD)                                                   \
          CSN_LOW();                                                         \
          *(uint8_t*)&SPI1->DR = (uint8_t)(CMD);                             \
          SPI_SLEEP_WHILE_XFER();                                            \
          if((uint8_t)SPI1->DR);                                             \
          CSN_HIGH()
            
__STATIC_INLINE uint8_t nrf_w_reg(uint8_t regNo, uint8_t regVal) {
  CSN_LOW();
  *(uint8_t*)&SPI1->DR = (uint8_t)(W_REGISTER | (REGISTER_MASK & regNo));
  SPI_SLEEP_WHILE_XFER();
  uint8_t r = (uint8_t)SPI1->DR;
  *(uint8_t*)&SPI1->DR = regVal;
  SPI_SLEEP_WHILE_XFER();
  if((uint8_t)SPI1->DR);
  CSN_HIGH();
  return r;
}
  
__STATIC_INLINE uint8_t nrf_r_reg(uint8_t regNo) {
  CSN_LOW();
  *(uint8_t*)&SPI1->DR = (uint8_t)(R_REGISTER | (REGISTER_MASK & regNo));
  SPI_SLEEP_WHILE_XFER();
  if((uint8_t)SPI1->DR);
  *(uint8_t*)&SPI1->DR = NOP;
  SPI_SLEEP_WHILE_XFER();
  CSN_HIGH();
  return (uint8_t)SPI1->DR;
}
            
#define NRF_WRITE_REG(REG, VAL)                                              \
          CSN_LOW();                                                         \
          *(uint8_t*)&SPI1->DR = (uint8_t)(W_REGISTER | (REGISTER_MASK & (REG)));\
          SPI_SLEEP_WHILE_XFER();                                            \
          if((uint8_t)SPI1->DR);   /* dummy read to reset flags */           \
          *(uint8_t*)&SPI1->DR = (uint8_t)(VAL);                             \
          SPI_SLEEP_WHILE_XFER();                                            \
          if((uint8_t)SPI1->DR);   /* dummy read to reset flags */           \
          CSN_HIGH()
            
#define NRF24_WRITE_PAYLOAD(PL, SIZE)                                        \
          CSN_LOW();                                                         \
                  /* - send the command: WRITE_PAYLOAD_WITH_NO_ACK - */      \
          *(uint8_t*)&SPI1->DR = (uint8_t)(W_TX_PAYLOAD_NOACK);              \
          SPI_SLEEP_WHILE_XFER();                                            \
          if((uint8_t)SPI1->DR);   /* dummy read to reset flags */           \
                  /* ---------- transfer payload ---------------*/           \
          for (uint8_t i = 0; i < SIZE; i++) {                               \
            *(uint8_t*)&SPI1->DR = ((uint8_t *)(&PL))[i];                    \
            SPI_SLEEP_WHILE_XFER();                                          \
            if((uint8_t)SPI1->DR); /* dummy read to reset flags */           \
          }                                                                  \
          CSN_HIGH()
            
#define NRF24_POWER_UP() \
          NRF_WRITE_REG(CONFIG, (      /* Turn ON transmitter              */\
            0 * NRF24_CONFIG_PRIM_RX   |  /* select TX mode                */\
            1 * NRF24_CONFIG_PWR_UP    |  /* turn power on                 */\
            1 * NRF24_CONFIG_EN_CRC    |  /* enable CRC                    */\
            1 * NRF24_CONFIG_CRCO      |  /* don't use 16-bit CRC          */\
            1 * NRF24_CONFIG_MAX_RT    |  /* disable MAX_RT flag           */\
            0 * NRF24_CONFIG_TX_DS     |  /* enable "TX Data Sent" flag    */\
            1 * NRF24_CONFIG_RX_DR        /* disable "RX Data Ready" flag  */\
          ))
            
#define NRF24_POWER_DOWN() \
          nrf_w_reg(CONFIG, (uint8_t)( /* Turn OFF transmitter             */\
            0 * NRF24_CONFIG_PRIM_RX   |  /* select TX mode                */\
            0 * NRF24_CONFIG_PWR_UP    |  /* turn power off                */\
            1 * NRF24_CONFIG_EN_CRC    |  /* enable CRC                    */\
            1 * NRF24_CONFIG_CRCO      |  /* don't use 16-bit CRC          */\
            1 * NRF24_CONFIG_MAX_RT    |  /* disable MAX_RT flag           */\
            1 * NRF24_CONFIG_TX_DS     |  /* disable "TX Data Sent" flag   */\
            1 * NRF24_CONFIG_RX_DR        /* disable "RX Data Ready" flag  */\
          ))

#define NRF24_RESET_STATUS() \
          nrf_w_reg(STATUS, (uint8_t)( /* clear status bits */               \
            1 * NRF24_STATUS_MAX_RT    |                                     \
            1 * NRF24_STATUS_TX_DS     |                                     \
            1 * NRF24_STATUS_RX_DR                                           \
          ))
      
#define TX_FIFO         FLUSH_TX
#define RX_FIFO         FLUSH_RX
            
#define NRF24_FLUSH(FIFO)    \
          NRF_WRITE_CMD(FIFO)

#define NRF24_OK ((uint8_t)(           \
          NRF24_STATUS_RX_P_NO_2 |     \
          NRF24_STATUS_RX_P_NO_1 |     \
          NRF24_STATUS_RX_P_NO_0       \
        ))                             
                                       
#define NRF24_TX_OK  ((uint8_t)(       \
          NRF24_OK               |     \
          NRF24_STATUS_TX_DS           \
        ))
            
#endif
