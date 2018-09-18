// #include <stdint.h>
// #include <stddef.h>
// #include <stdbool.h>
#include "stm32f0xx.h"
#include "gpio.h"
// #include "control.h"
#include "adc.h"
// #include "spi.h"
#include "tim.h"
// #include "gpio.h"
// #include "nrf24l01.h"
// #include "flash.h"

uint32_t i2c_status = 0;
uint32_t nvStatus; // non-volatile system status
uint16_t flash_status = 0;

void __STATIC_INLINE check_vdd(void) {
  RCC->APB1ENR = RCC_APB1ENR_PWREN;        // enable clock for PWR
  PWR->CR = PWR_CR_PVDE | PWR_CR_PLS;      // enable PVD and set all PLS bits high
  RUN_MCU_AT(LOWEST_FREQ);                 // set lowest MCU speed
  RUN_MCU_AT(EIGHT_MHZ);                   // restore normal MCU speed
  while((PWR->CSR & PWR_CSR_PVDO) != 0);   // wait till VDD becomes high
  PWR->CR = 0;                             // disable PVD
  RCC->APB1ENR = 0;                        // disable clock for PWR
}

void turnRegulatorOn(void) {
  
  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( // Configure GPIOA
#ifndef SWD_DISABLED      
    PIN_CONF(PIN(13), PINV_ALT_FUNC) |    // PA13 AF0 -- SYS_SWDIO
    PIN_CONF(PIN(14), PINV_ALT_FUNC) |    // PA14 AF0 -- SYS_SWDCLK
#endif
    PIN_CONF(PIN(0), PINV_OUTPUT)         // PA0 OUT  -- DC-DC (LOW)
  );
  
  check_vdd();                            // check VDD is HIGH
}

void turnRegulatorOff(void) {

  // GPIOA->BSRR = GPIO_BSRR_BS_0;            // set PA0 to 1
  TOGGLE_PIN(A, 0, HIGH);
  
  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( // Configure GPIOA
#ifndef SWD_DISABLED      
    PIN_CONF(PIN(13), PINV_ALT_FUNC) |     // PA13 AF0 -- SYS_SWDIO
    PIN_CONF(PIN(14), PINV_ALT_FUNC) |     // PA14 AF0 -- SYS_SWDCLK
#endif
    PIN_CONF(PIN(0), PINV_OUTPUT)          // PA0 OUT  -- DC-DC (HIGH)
  );  
  RUN_MCU_AT(LOWEST_FREQ);                 // set lowest MCU speed
  RUN_MCU_AT(EIGHT_MHZ);                   // restore normal MCU speed
}

uint8_t powerCycle(void) {

  RCC->AHBENR = (  
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    | // enable clock for FLASH (while sleep) in debug mode
    RCC_AHBENR_SRAMEN     | // enable clock for SRAM (while sleep) in debug mode
#endif
    RCC_AHBENR_GPIOAEN    | // enable clock for GPIOA
    RCC_AHBENR_GPIOFEN      // enable clock for GPIOF
  );
  
  /* -- set PF0 HIGH to turn load switch OFF -- */
  
  GPIOF->ODR = GPIO_ODR_0;  // prepare PF0 to set pin HIGH
  GPIOF->MODER = ANALOG_MODE_FOR_ALL_PINS - PIN_CONF(PIN(0), PINV_OUTPUT); // set PF0 pin HIGH
  
  // set PA9 and PA10 LOW to pull down V_Load rail
  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( // Configure GPIOA
#ifndef SWD_DISABLED
    PIN_CONF(PIN(13), PINV_ALT_FUNC) |  // PA13 AF0 -- SYS_SWDIO
    PIN_CONF(PIN(14), PINV_ALT_FUNC) |  // PA14 AF0 -- SYS_SWDCLK
#endif
    PIN_CONF(PIN(2), PINV_OUTPUT)    |  // PA2  OUT -- I2C_SCL (LOW)
    PIN_CONF(PIN(9), PINV_OUTPUT)    |  // PA9  OUT -- I2C_SCL (LOW)
    PIN_CONF(PIN(10), PINV_OUTPUT)      // PA10 OUT -- I2C_SDA (LOW)
  );
  
  GPIOA->BSRR = GPIO_BSRR_BS_2;         // Set PA2 high -- turn On powCyc LED

  S_DELAY(428);                         // (428*2+4)*512/8000 = ~55.04 ms

  while(adc_read_vload() > 50);         // wait till vload rail drops below 50mv
  
  GPIOA->BSRR = GPIO_BSRR_BR_2;         // Set PA2 low  -- turn Off powCyc LED

  // turn DC-DC converter on (pull down PA0)
  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( // Configure GPIOA
#ifndef SWD_DISABLED      
    PIN_CONF(PIN(13), PINV_ALT_FUNC) |  // PA13 AF0 -- SYS_SWDIO
    PIN_CONF(PIN(14), PINV_ALT_FUNC) |  // PA14 AF0 -- SYS_SWDCLK
#endif
    PIN_CONF(PIN(0), PINV_OUTPUT)    |  // PA0  OUT -- DC_SWITCH (LOW)
    PIN_CONF(PIN(9), PINV_OUTPUT)    |  // PA9  OUT -- I2C_SCL (LOW)
    PIN_CONF(PIN(10), PINV_OUTPUT)      // PA10 OUT -- I2C_SDA (LOW)
  );

  // wait till VDD becomes high
  
  check_vdd();

  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( // Configure GPIOA in analog mode
#ifndef SWD_DISABLED
    PIN_CONF(PIN(13), PINV_ALT_FUNC) |  // PA13 AF0 -- SYS_SWDIO
    PIN_CONF(PIN(14), PINV_ALT_FUNC) |  // PA14 AF0 -- SYS_SWDCLK
#endif
    0
  );

  GPIOF->BRR = GPIO_BRR_BR_0;          // set PF0 LOW to turn load switch ON
  
  while (adc_read_vload() < 240);      // ensure vload rail is HIGH

  RCC->AHBENR = (     // prune the clock
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    |
    RCC_AHBENR_SRAMEN     |
    RCC_AHBENR_GPIOAEN    |  // GPIOA clock enable
#endif
    0
  );
  
  nvStatus |= NV_LSI_CALIBRATION_REQ; // set LSI calibration request

  return 0;
}  

void instant_standby(void) {
  RCC->APB1ENR = 0;
  RCC->APB2ENR = 0;

  RCC->AHBENR = (     // prune the clock
#ifndef SWD_DISABLED
    RCC_AHBENR_SRAMEN     | 
    RCC_AHBENR_FLITFEN    | 
    RCC_AHBENR_GPIOAEN    |
#endif
    0
  );
  
  RCC->CSR &= ~RCC_CSR_LSION;
  SCB->SCR = SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SEVONPEND_Msk;
  
  PWR->CR = (
    PWR_CR_PDDS      |                   // set PPDS bit (1 = Standby, 0 = Stop)
    PWR_CR_CSBF      |                   // clear Standby flag
    PWR_CR_CWUF                          // clear wakeup flag
  );                                     
  
  RUN_MCU_AT(LOWEST_FREQ);               // set lowest MCU speed
  while (1) {
    __SEV();
    __WFE();
    __WFE();
  }
}

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

void NMI_Handler(void) {
  instant_standby();
}

void HardFault_Handler(void) {
  instant_standby();
}

void SVC_Handler(void) {
  instant_standby();
}

void PendSV_Handler(void) {
  instant_standby();
}
