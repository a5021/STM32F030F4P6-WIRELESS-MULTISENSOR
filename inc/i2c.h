#ifndef __I2C_INCLUDED
#define __I2C_INCLUDED

#include <stdint.h>
#include <stdbool.h>

#ifdef __ICCARM__
  #include "intrinsics.h"
#endif

#include "gpio.h"
#include "tim.h"
#include "control.h"

extern uint32_t i2c_status;

#define  ERROR_I2C_NO_ACK               (uint32_t) (1<<0)  /* 0x01 */
#define  ERROR_I2C_STOP_TIMEOUT         (uint32_t) (1<<1)  /* 0x02 */
#define  ERROR_I2C_TX_TIMEOUT           (uint32_t) (1<<2)  /* 0x04 */
#define  ERROR_I2C_RX_TIMEOUT           (uint32_t) (1<<3)  /* 0x08 */
#define  ERROR_I2C_BUS_ERROR            (uint32_t) (1<<4)  /* 0x10 */
#define  ERROR_I2C_ARBITRATION_LOST     (uint32_t) (1<<5)  /* 0x11 */
#define  ERROR_I2C_DMA_TRANSFER_FAIL    (uint32_t) (1<<6)  /* 0x12 */
#define  ERROR_I2C_UNKNOWN              (uint32_t) (1<<7)  /* 0x14 */

#define ERROR_VAR                       i2c_status
#define ABANDON_ROUTINE(ERR_FLAG)       ERROR_VAR |= (ERR_FLAG); return false

#define I2C_CR1_RX()      I2C1->CR1 = (I2C_CR1_PE | I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE)
#define I2C_CR1_TX()      I2C1->CR1 = (I2C_CR1_PE | I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE)
#define I2C_CR1_STOP()    I2C1->ICR = I2C_ICR_STOPCF; I2C1->CR1 = (I2C_CR1_PE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_ERRIE)
#define I2C_CR1_DMA_RX()  I2C1->CR1 = (I2C_CR1_PE | I2C_CR1_NACKIE | I2C_CR1_ERRIE | I2C_CR1_RXDMAEN)
#define I2C_CR1_DMA_TX()  I2C1->CR1 = (I2C_CR1_PE | I2C_CR1_NACKIE | I2C_CR1_ERRIE | I2C_CR1_TXDMAEN)

#define I2C_ERROR_MASK    (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO)

#define SLEEP_MODE()                                              \
          do {                                                    \
            NVIC_ClearPendingIRQ(I2C1_IRQn);                      \
            uint32_t rcc_cfgr = RCC->CFGR;                        \
            RUN_MCU_AT(HF_MHZ);                                   \
            __WFE();                                              \
            RCC->CFGR = rcc_cfgr;                                 \
          } while(0)

#define I2C_CHECK_FOR_BUS_FAIL()                                  \
          if ((I2C1->ISR & I2C_ERROR_MASK) != 0) {                \
            uint32_t i2c_err = I2C1->ISR & I2C_ERROR_MASK;        \
            I2C1->ICR = (                                         \
              I2C_ICR_NACKCF |                                    \
              I2C_ICR_BERRCF |                                    \
              I2C_ICR_ARLOCF                                      \
            );                                                    \
            I2C1->CR1 &= ~I2C_CR1_PE;                             \
            while ((I2C1->CR1 & I2C_CR1_PE) != 0);                \
            I2C1->CR1 |= I2C_CR1_PE;                              \
            ABANDON_ROUTINE(i2c_err >> 4);                        \
          }

#define I2C_START_WRITING(ADDR, COUNT, VALUE)                     \
          I2C1->TXDR = VALUE;                                     \
          I2C1->CR2 = (                                           \
            I2C_CR2_AUTOEND |                                     \
            ((COUNT) << 16) |                                     \
            ((ADDR) << 1)   |                                     \
            I2C_CR2_START                                         \
          )

#define I2C_START_READING(ADDR, COUNT)                            \
          I2C1->CR2 = (                                           \
            I2C_CR2_AUTOEND           |                           \
            (((uint32_t)COUNT) << 16) |                           \
            I2C_CR2_RD_WRN            |                           \
            ((ADDR) << 1)             |                           \
            I2C_CR2_START                                         \
          )

#define I2C_WAIT_FOR_STOP_FLAG()                                  \
          while((I2C1->ISR & I2C_ISR_STOPF) != I2C_ISR_STOPF) {   \
            I2C_CHECK_FOR_BUS_FAIL();                             \
          }                                                       \
          I2C1->ICR = I2C_ICR_STOPCF

#define I2C_SLEEP_UNTIL_STOP()                                    \
          SLEEP_MODE();                                           \
          I2C_CHECK_FOR_BUS_FAIL();                               \
          if((I2C1->ISR & I2C_ISR_STOPF) != I2C_ISR_STOPF) {      \
            ABANDON_ROUTINE(ERROR_I2C_UNKNOWN);                   \
          }                                                       \
          I2C1->ICR = I2C_ICR_STOPCF

#define I2C_SLEEP_UNTIL_DMA_TC()                                  \
          SLEEP_MODE();                                           \
          I2C_CHECK_FOR_BUS_FAIL();                               \
          if ((DMA1->ISR & DMA_ISR_TCIF3) != DMA_ISR_TCIF3) {     \
            if ((DMA1->ISR & DMA_ISR_TEIF3) == DMA_ISR_TEIF3) {   \
              ABANDON_ROUTINE(ERROR_I2C_DMA_TRANSFER_FAIL);       \
            } else {                                              \
              ABANDON_ROUTINE(ERROR_I2C_UNKNOWN);                 \
            }                                                     \
          }                                                       \
          I2C1->ICR = I2C_ICR_STOPCF

#define I2C_WAIT_FOR_RX_FLAG()                                    \
          while((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {     \
            I2C_CHECK_FOR_BUS_FAIL();                             \
          }

#define I2C_SLEEP_UNTIL_RXNE()                                    \
          do {                                                    \
            SLEEP_MODE();                                         \
            if((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {      \
              I2C_CHECK_FOR_BUS_FAIL();                           \
              ABANDON_ROUTINE(ERROR_I2C_UNKNOWN);                 \
            }                                                     \
          } while(0)

#define I2C_WAIT_FOR_TX_FLAG()                                    \
          while((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {       \
            I2C_CHECK_FOR_BUS_FAIL();                             \
          }

#define I2C_SLEEP_UNTIL_TXE()                                     \
          do {                                                    \
            SLEEP_MODE();                                         \
            if((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {        \
              I2C_CHECK_FOR_BUS_FAIL();                           \
              ABANDON_ROUTINE(ERROR_I2C_UNKNOWN);                 \
            }                                                     \
          } while(0)

__STATIC_INLINE void i2c_reset(void) {
//
//  Software reset
//  ==============
//
//  A software reset can be performed by clearing the PE bit
//  in the I2C_CR1 register. In that case I2C lines SCL and SDA
//  are released. Internal states machines are reset and
//  communication control bits, as well as status bits come back
//  to their reset value. The configuration registers are not impacted.
//  Here is the list of impacted register bits:
//  1. I2C_CR2 register: START, STOP, NACK
//  2. I2C_ISR register: BUSY, TXE, TXIS, RXNE, ADDR, NACKF,
//                       TCR, TC, STOPF, BERR, ARLO, OVR
//  and in addition when the SMBus feature is supported:
//  1. I2C_CR2 register: PECBYTE
//  2. I2C_ISR register: PECERR, TIMEOUT, ALERT
//
//  PE must be kept low during at least 3 APB clock cycles in order
//  to perform the software reset. This is ensured by writing the
//  following software sequence: - Write PE=0 - Check PE=0 - Write PE=1

  I2C1->CR1 &= (uint32_t)~((uint32_t)I2C_CR1_PE);  // disable peripheral
  while ((I2C1->CR1 & I2C_CR1_PE) == I2C_CR1_PE);  // ensure PE = 0
  I2C1->CR1 |= I2C_CR1_PE;                         // enable peripheral
}

__STATIC_INLINE void i2c_disable(void) {
  I2C1->CR1 &= ~I2C_CR1_PE;             // I2C1 Peripheral disable

  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( // Configure GPIOA
#ifndef SWD_DISABLED
    PIN_CONF(PIN(13), PINV_ALT_FUNC) |  // PA13 AF0 -- SYS_SWDIO
    PIN_CONF(PIN(14), PINV_ALT_FUNC) |  // PA14 AF0 -- SYS_SWDCLK
#endif
    0
  );

  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN      | // enable clock for FLASH in sleep mode to allow debug
    RCC_AHBENR_SRAMEN       | // enable clock for SRAM in sleep mode to allow debug
    RCC_AHBENR_GPIOAEN      | // enable clock for GPIOA
#endif
    0                         // disable clock for GPIOA
  );

  RCC->APB1ENR = 0;           // disable clock for I2C1
}

__STATIC_INLINE void i2c_enable(void) {

  RCC->APB1ENR = RCC_APB1ENR_I2C1EN;    // enable clock for I2C1
  I2C1->CR1 |= I2C_CR1_PE;              // enable I2C1 Peripheral

  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN      | // enable clock for FLASH in sleep mode to allow debug
    RCC_AHBENR_SRAMEN       | // enable clock for SRAM in sleep mode to allow debug
#endif
    RCC_AHBENR_GPIOAEN        // enable clock for GPIOA
  );

  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( // Configure GPIOA
#ifndef SWD_DISABLED
    PIN_CONF(PIN(13), PINV_ALT_FUNC) |  // PA13: AF0 -- SYS_SWDIO
    PIN_CONF(PIN(14), PINV_ALT_FUNC) |  // PA14: AF0 -- SYS_SWDCLK
#endif
    PIN_CONF(PIN(9), PINV_ALT_FUNC)  |  // PA9:  AF4 -- I2C_SCL
    PIN_CONF(PIN(10), PINV_ALT_FUNC)    // PA10: AF4 -- I2C_SDA
  );
}
/*
__STATIC_INLINE void sleep_delay(uint16_t cnt) {
  i2c_disable();
  S_DELAY(cnt);  // (cnt*2+4)*512/8000 ms
  i2c_enable();
}
*/
__STATIC_INLINE bool i2c_sleep(uint32_t cnt) {
  I2C1->CR1 &= ~I2C_CR1_PE;             // I2C1 Peripheral disable

  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( // Configure GPIOA
#ifndef SWD_DISABLED
    PIN_CONF(PIN(13), PINV_ALT_FUNC) |  // PA13 AF0 -- SYS_SWDIO
    PIN_CONF(PIN(14), PINV_ALT_FUNC) |  // PA14 AF0 -- SYS_SWDCLK
#endif
    0
  );

  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN      | // enable clock for FLASH in sleep mode to allow debug
    RCC_AHBENR_SRAMEN       | // enable clock for SRAM in sleep mode to allow debug
    RCC_AHBENR_GPIOAEN      | // enable clock for GPIOA
#endif
    0                         // disable clock for GPIOA
  );

  RCC->APB1ENR = RCC_APB1ENR_TIM14EN;  // disable clock for I2C1

  TIM14->CR1 = TIM_CR1_URS;
  TIM14->PSC = cnt - 1;
  TIM14->ARR = 1;
  TIM14->EGR = TIM_EGR_UG;
  TIM14->DIER = TIM_DIER_UIE;
  TIM14->SR = 0;
  NVIC_ClearPendingIRQ(TIM14_IRQn);

  uint32_t rcc_cfgr = RCC->CFGR;
  RUN_MCU_AT(LOWEST_FREQ);
  TIM14->CR1 = TIM_CR1_CEN;   // Run timer

  __WFE();

  RCC->CFGR = rcc_cfgr;
  RCC->APB1RSTR = RCC_APB1RSTR_TIM14RST;
  RCC->APB1ENR = RCC_APB1ENR_I2C1EN;    // enable clock for I2C1
  i2c_enable();
  RCC->APB1RSTR = 0;

//  RCC->APB1ENR = RCC_APB1ENR_I2C1EN;    // enable clock for I2C1
//  I2C1->CR1 |= I2C_CR1_PE;              // enable I2C1 Peripheral
//
//  RCC->AHBENR = (
//#ifndef SWD_DISABLED
//    RCC_AHBENR_FLITFEN      | // enable clock for FLASH in sleep mode to allow debug
//    RCC_AHBENR_SRAMEN       | // enable clock for SRAM in sleep mode to allow debug
//#endif
//    RCC_AHBENR_GPIOAEN        // enable clock for GPIOA
//  );
//
//  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( // Configure GPIOA
//#ifndef SWD_DISABLED
//    PIN_CONF(PIN(13), PINV_ALT_FUNC) |  // PA13: AF0 -- SYS_SWDIO
//    PIN_CONF(PIN(14), PINV_ALT_FUNC) |  // PA14: AF0 -- SYS_SWDCLK
//#endif
//    PIN_CONF(PIN(9), PINV_ALT_FUNC)  |  // PA9:  AF4 -- I2C_SCL
//    PIN_CONF(PIN(10), PINV_ALT_FUNC)    // PA10: AF4 -- I2C_SDA
//  );
  return true;
}

#ifndef I2C_CR1_DNF
  #define I2C_CR1_DNF I2C_CR1_DFN
#endif

__STATIC_INLINE void initI2C(void)  {

  RCC->APB1ENR = RCC_APB1ENR_I2C1EN;     // I2C1 clock enable
  I2C1->TIMINGR = (uint32_t)0x0010020A;  // set I2C interface to 400kHz mode

  I2C1->CR1 = (      // I2C Control register 1
    1 * I2C_CR1_PE          |   // Peripheral enable
    1 * I2C_CR1_TXIE        |   // TX interrupt enable
    1 * I2C_CR1_RXIE        |   // RX interrupt enable
    0 * I2C_CR1_ADDRIE      |   // Address match interrupt enable
    1 * I2C_CR1_NACKIE      |   // NACK received interrupt enable
    1 * I2C_CR1_STOPIE      |   // STOP detection interrupt enable
    0 * I2C_CR1_TCIE        |   // Transfer complete interrupt enable
    1 * I2C_CR1_ERRIE       |   // Errors interrupt enable
    0 * I2C_CR1_DNF         |   // Digital noise filter
    0 * I2C_CR1_ANFOFF      |   // Analog noise filter OFF
    0 * I2C_CR1_SWRST       |   // Software reset
    0 * I2C_CR1_TXDMAEN     |   // DMA transmission requests enable
    0 * I2C_CR1_RXDMAEN     |   // DMA reception requests enable
    0 * I2C_CR1_SBC         |   // Slave byte control
    0 * I2C_CR1_NOSTRETCH   |   // Clock stretching disable
    0 * I2C_CR1_WUPEN       |   // Wakeup from STOP enable
    0 * I2C_CR1_GCEN        |   // General call enable
    0 * I2C_CR1_SMBHEN      |   // SMBus host address enable
    0 * I2C_CR1_SMBDEN      |   // SMBus device default address enable
    0 * I2C_CR1_ALERTEN     |   // SMBus alert enable
    0 * I2C_CR1_PECEN           // PEC enable
  );

  /********************************************************************
  *
  *         Configure I2C pins
  *
  ********************************************************************/

  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN      |
    RCC_AHBENR_SRAMEN       |
#endif
    RCC_AHBENR_GPIOAEN          // enable clock for GPIOA
  );

    //0: Output push-pull (reset state)
    //1: Output open-drain
  GPIOA->OTYPER = (     // GPIO port output type register
    1 * GPIO_OTYPER_OT_9   |    // set open drain mode for PA9 (I2C_SCL)
    1 * GPIO_OTYPER_OT_10       // set open drain mode for PA10 (I2C_SDA)
  );

  // Configure alternate functions
  // Only I2C pins (PA9 and PA10) to be configured as alternate func.
  //    (SWD pins are in alternate func. mode by default)

  GPIOA->AFR[1] = (     // with High 8 bits of AFR
    PIN_AF(PIN(9), AF(4))  |  // PA9:  AF4 I2C_SCL
    PIN_AF(PIN(10), AF(4))    // PA10: AF4 I2C_SDA
  );

  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( // Configure GPIOA
#ifndef SWD_DISABLED
    PIN_CONF(PIN(13), PINV_ALT_FUNC) |  // PA13: AF0 -- SYS_SWDIO
    PIN_CONF(PIN(14), PINV_ALT_FUNC) |  // PA14: AF0 -- SYS_SWDCLK
#endif
    PIN_CONF(PIN(9), PINV_ALT_FUNC)  |  // PA9:  AF4 -- I2C_SCL
    PIN_CONF(PIN(10), PINV_ALT_FUNC)    // PA10: AF4 -- I2C_SDA
  );
}

#endif
