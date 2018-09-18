// #include <stddef.h>
#include "stm32f0xx.h"
#include "adc.h"
#include "gpio.h"
#include "control.h"

#define ADC_VREF_CHANNEL   ADC_CHSELR_CHSEL17
#define ADC_TS_CHANNEL     ADC_CHSELR_CHSEL16
#define ADC_VBAT_CHANNEL   ADC_CHSELR_CHSEL1
#define ADC_VLOAD_CHANNEL  ADC_CHSELR_CHSEL9

//#define TEMP110_CAL_ADDR   ((uint16_t*) 0x1FFFF7C2)
//#define TEMP30_CAL_ADDR    ((uint16_t*) (0x1ffff7b8))
//#define VREFINT_CAL_ADDR   ((uint16_t*) 0x1FFFF7BA)
//
//#define VDD_CALIB          ((uint16_t)  (330))

uint16_t vref_buf[ADC_BUF_SIZE + 2], ts_buf[ADC_BUF_SIZE], v_buf[ADC_BUF_SIZE];

// uint16_t vref_buf[ADC_BUF_SIZE + 2], ts_buf[ADC_BUF_SIZE], v_buf[ADC_BUF_SIZE];

//void __STATIC_INLINE calibrateADC(void) {
//  // calibrate ADC
//  if ((ADC1->CR & ADC_CR_ADEN) != 0) {         // if ADC enabled
//    ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);      // turn it off
//  }
//  ADC1->CR |= ADC_CR_ADCAL;                    // launch the calibration 
//  while ((ADC1->CR & ADC_CR_ADCAL)!= 0);       // Wait until ADCAL=0
//}

void __STATIC_INLINE initADC(bool use_ts) {

  if (use_ts) {
    // ADC common configuration register
    // Address offset: 0x308
    // Reset value: 0x0000 0000  
    ADC->CCR = (
      1 * ADC_CCR_TSEN       |    // Tempurature sensore enable
      1 * ADC_CCR_VREFEN          // Vrefint enable
    );
  } else {
    ADC->CCR = (
      0 * ADC_CCR_TSEN       |    // Tempurature sensore enable
      1 * ADC_CCR_VREFEN          // Vrefint enable
    );
  }  
  // ADC configuration register 1
  // ============================
  // Address offset: 0x0C
  // Reset value: 0x0000 0000
  ADC1->CFGR1 = (
      // AWDCH[4:0] bits (Analog watchdog channel select bits)
    0 * ADC_CFGR1_AWDCH_0  |    // Bit 0
    0 * ADC_CFGR1_AWDCH_1  |    // Bit 1
    0 * ADC_CFGR1_AWDCH_2  |    // Bit 2
    0 * ADC_CFGR1_AWDCH_3  |    // Bit 3
    0 * ADC_CFGR1_AWDCH_4  |    // Bit 4
      
    0 * ADC_CFGR1_AWDEN    |    // Analog watchdog enable on regular channels
    0 * ADC_CFGR1_AWDSGL   |    // Enable the watchdog on a single channel or on all channels
    0 * ADC_CFGR1_DISCEN   |    // Discontinuous mode on regular channels
    1 * ADC_CFGR1_AUTOFF   |    // ADC auto power off
    0 * ADC_CFGR1_WAIT     |    // ADC wait conversion mode
    1 * ADC_CFGR1_CONT     |    // Continuous Conversion
    0 * ADC_CFGR1_OVRMOD   |    // Overrun mode
      
      // EXTEN[1:0] bits (External Trigger Conversion mode for regular channels)
    0 * ADC_CFGR1_EXTEN_0  |    // Bit 0
    0 * ADC_CFGR1_EXTEN_1  |    // Bit 1
      
      // EXTSEL[2:0] bits (External Event Select for regular group)
    0 * ADC_CFGR1_EXTSEL_0 |    // Bit 0
    0 * ADC_CFGR1_EXTSEL_1 |    // Bit 1
    0 * ADC_CFGR1_EXTSEL_2 |    // Bit 2
      
    0 * ADC_CFGR1_ALIGN    |    // Data Alignment
      
      // RES[1:0] bits (Resolution)
    0 * ADC_CFGR1_RES_0    |    // Bit 0
    0 * ADC_CFGR1_RES_1    |    // Bit 1
      
    0 * ADC_CFGR1_SCANDIR  |    // Sequence scan direction
    0 * ADC_CFGR1_DMACFG   |    // Direct memory access configuration
    1 * ADC_CFGR1_DMAEN         // Direct memory access enable
  );
  
  // ADC sampling time register
  // ==========================
  // Address offset: 0x14
  // Reset value: 0x0000 0000
  
  // These bits are written by software to select the sampling time 
  // that applies to all channels.
  // 000: 1.5 ADC clock cycles
  // 001: 7.5 ADC clock cycles
  // 010: 13.5 ADC clock cycles
  // 011: 28.5 ADC clock cycles
  // 100: 41.5 ADC clock cycles
  // 101: 55.5 ADC clock cycles
  // 110: 71.5 ADC clock cycles
  // 111: 239.5 ADC clock cycles
  // Note: Software is allowed to write these bits only when ADSTART=0
  // (which ensures that no conversion is ongoing)  
  ADC1->SMPR = (       // Sampling time = 239.5 ADC clock cycles = 239.5/14mhz = 17.1uS 
    1 * ADC_SMPR_SMP_0 |           
    1 * ADC_SMPR_SMP_1 |           
    1 * ADC_SMPR_SMP_2 
  );

  // ADC1->TR;           // ADC watchdog threshold register

  // ADC channel selection register
  // ==============================
  // Address offset: 0x28
  // Reset value: 0x0000 0000 
  // Note: Software is allowed to write these bits only when ADSTART=0 
  // (which ensures that no conversion is ongoing).
  ADC1->CHSELR = (
    0 * ADC_CHSELR_CHSEL18 |    // Channel 18 selection
    1 * ADC_CHSELR_CHSEL17 |    // Channel 17 selection (Vref)
    0 * ADC_CHSELR_CHSEL16 |    // Channel 16 selection
    0 * ADC_CHSELR_CHSEL15 |    // Channel 15 selection
    0 * ADC_CHSELR_CHSEL14 |    // Channel 14 selection
    0 * ADC_CHSELR_CHSEL13 |    // Channel 13 selection
    0 * ADC_CHSELR_CHSEL12 |    // Channel 12 selection
    0 * ADC_CHSELR_CHSEL11 |    // Channel 11 selection
    0 * ADC_CHSELR_CHSEL10 |    // Channel 10 selection
    0 * ADC_CHSELR_CHSEL9  |    // Channel 9 selection
    0 * ADC_CHSELR_CHSEL8  |    // Channel 8 selection
    0 * ADC_CHSELR_CHSEL7  |    // Channel 7 selection
    0 * ADC_CHSELR_CHSEL6  |    // Channel 6 selection
    0 * ADC_CHSELR_CHSEL5  |    // Channel 5 selection
    0 * ADC_CHSELR_CHSEL4  |    // Channel 4 selection
    0 * ADC_CHSELR_CHSEL3  |    // Channel 3 selection
    0 * ADC_CHSELR_CHSEL2  |    // Channel 2 selection
    0 * ADC_CHSELR_CHSEL1  |    // Channel 1 selection
    0 * ADC_CHSELR_CHSEL0       // Channel 0 selection
  );

  ADC1->CR |= ADC_CR_ADCAL;                    // launch the calibration 
  while ((ADC1->CR & ADC_CR_ADCAL)!= 0);       // Wait until ADCAL=0
}

#define DMA_CH1_ENABLE(BUF, SIZE)                                             \
          DMA1_Channel1->CMAR = (uint32_t)(BUF);                              \
          DMA1_Channel1->CNDTR = (SIZE);                                      \
          DMA1_Channel1->CCR = (                                              \
            1 * DMA_CCR_EN   |      /* Channel enable                       */\
            1 * DMA_CCR_TCIE |      /* Transfer complete interrupt enable   */\
            0 * DMA_CCR_HTIE |      /* Half Transfer interrupt enable       */\
            0 * DMA_CCR_TEIE |      /* Transfer error interrupt enable      */\
            0 * DMA_CCR_DIR  |      /* Data transfer direction              */\
            0 * DMA_CCR_CIRC |      /* Circular mode                        */\
            0 * DMA_CCR_PINC |      /* Peripheral increment mode            */\
            1 * DMA_CCR_MINC |      /* Memory increment mode                */\
             /*                                                             */\
             /* PSIZE[1:0] bits (Peripheral size)                           */\
             /*       00: 8-bits                                            */\
             /*       01: 16-bits                                           */\
             /*       10: 32-bits                                           */\
             /*       11: Reserved                                          */\
             /*                                                             */\
            1 * DMA_CCR_PSIZE_0 |   /* Bit 0                                */\
            0 * DMA_CCR_PSIZE_1 |   /* Bit 1                                */\
             /*                                                             */\
             /* MSIZE[1:0] bits (Memory size)                               */\
             /*       00: 8-bits                                            */\
             /*       01: 16-bits                                           */\
             /*       10: 32-bits                                           */\
             /*       11: Reserved                                          */\
             /*                                                             */\
            1 * DMA_CCR_MSIZE_0 |   /* Bit 0                                */\
            0 * DMA_CCR_MSIZE_1 |   /* Bit 1                                */\
             /*                                                             */\
             /* PL[1:0] bits(Channel Priority level)                        */\
             /*       00: Low                                               */\
             /*       01: Medium                                            */\
             /*       10: High                                              */\
             /*       11: Very high                                         */\
            0 * DMA_CCR_PL_0    |   /* Bit 0                                */\
            1 * DMA_CCR_PL_1    |   /* Bit 1                                */\
            0 * DMA_CCR_MEM2MEM     /* Memory to memory mode                */\
          );                                                                  \
          NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn) 

#define DMA_CH1_DISABLE() DMA1->IFCR = (                                      \
          DMA_IFCR_CGIF1    |                                                 \
          DMA_IFCR_CTCIF1   |                                                 \
          DMA_IFCR_CHTIF1   |                                                 \
          DMA_IFCR_CTEIF1                                                     \
        );                                                                    \
        DMA1_Channel1->CCR = 0                                  

#define START_ADC_CONVERSION()  ADC1->CR = ADC_CR_ADSTART;
  
uint32_t adc_read_vload(void) {

  RCC->CR2 |= RCC_CR2_HSI14ON;          // start HSI14 RC oscillator
  RCC->APB2ENR = RCC_APB2ENR_ADC1EN;    // enable clock of the ADC
  while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0);  // Wait HSI14 is ready
  
  initADC(false);      // false = w/o temerature sensor
  
  RCC->AHBENR |= (                      // enable clock for
    RCC_AHBENR_DMA1EN |                 // DMA peripheral
    RCC_AHBENR_SRAMEN                   // SRAM interface in sleep mode
  );
  
  DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR)); // set channel source
  
  DMA_CH1_ENABLE(vref_buf, ADC_BUF_SIZE + 2); // set destination and size of transfer
  
  uint32_t rcc_cfgr = RCC->CFGR;        // save current MCU speed
  
  START_ADC_CONVERSION();               // run ADC
  
  RUN_MCU_AT(HF_MHZ);                   // lower MCU speed
  __WFE();                              // put MCU into the sleep mode
  RCC->CFGR = rcc_cfgr;                 // restore MCU speed

  ADC->CCR &= ~ADC_CCR_VREFEN;          // turn vref off

  DMA_CH1_DISABLE();                    // complete transfer

  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;    // enable clock for GPIOB
  GPIOB->MODER = ANALOG_MODE_FOR_ALL_PINS;
  
  ADC1->CHSELR = ADC_VLOAD_CHANNEL;     // select AIN9 (PB1)
  
  ADC1->SMPR = (                        // configure sampling time
    1 * ADC_SMPR_SMP_0 |                // = 7.5 ADC clock cycles
    0 * ADC_SMPR_SMP_1 |           
    0 * ADC_SMPR_SMP_2 
  );  

  DMA_CH1_ENABLE(v_buf, ADC_BUF_SIZE);  // set destination and size of transfer
  
  START_ADC_CONVERSION();               // run ADC
  __WFE();                              // put MCU into the sleep mode
    
  DMA_CH1_DISABLE();                    // complete transfer
  
  RCC->AHBENR &= ~(  // disable clock for peripherals used in the function
    RCC_AHBENR_DMA1EN  | 
    RCC_AHBENR_GPIOBEN |
    RCC_AHBENR_SRAMEN
  );
  
  RCC->APB2ENR = 0;                      // disable clock for ADC
  RCC->CR2 &= ~RCC_CR2_HSI14ON;          // stop HSI14 RC oscillator
  
  uint32_t vref = 0, vload = 0;
  for (uint32_t i = 0; i < ADC_BUF_SIZE; i++) {
    vref += ((uint16_t*)&vref_buf[2])[i];
    vload += v_buf[i];
  }
  vref = VREFINT_CAL * VDD_CALIB / (vref / 16);
  vload /= 16;

  if (4095 == vload) {            /* check if it is equal to VREF */
    return vref;                  /* return VREF value            */
  }                               /* recalc v_bat value           */
  vload *= vref;                  /* v = adc * V_REF / 4095       */
  vload /= 4095;
                                                           
  return (uint16_t) vload;
}    

bool adc_get_data(void) {

  RCC->CR2 |= RCC_CR2_HSI14ON;          // start HSI14 RC oscillator
  RCC->APB2ENR = RCC_APB2ENR_ADC1EN;    // enable clock for ADC
  while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0);  // Wait HSI14 is ready
  
  initADC(true);
  
  RCC->AHBENR = (      // configure clock gating
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    |
    RCC_AHBENR_GPIOAEN    |
#endif
    RCC_AHBENR_SRAMEN     |             // enable clock for SRAM (in sleep) 
    RCC_AHBENR_DMA1EN                   // enable clock for DMA1 
  );

  DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR));
  
  DMA_CH1_ENABLE(vref_buf, ADC_BUF_SIZE + 2); // configure DMA to get VREF data
  
  START_ADC_CONVERSION();

  register uint32_t r = RCC->CFGR;      // save CPU speed value
  RUN_MCU_AT(HF_MHZ);                   // slow down CPU
  __WFE();                              // sleep
  RCC->CFGR = r;                        // restore CPU speed
  
  ADC->CCR = ADC_CCR_TSEN;              // turn vref off
  
  DMA_CH1_DISABLE();

  ADC1->CHSELR = ADC_TS_CHANNEL;        // select temperature sensor channel
  DMA_CH1_ENABLE(ts_buf, ADC_BUF_SIZE); // set DMA to get temp sensor data
  
  START_ADC_CONVERSION();
  RUN_MCU_AT(HF_MHZ);                   // slow down CPU
  __WFE();                              // sleep
  RUN_MCU_AT(EIGHT_MHZ);                // restore CPU speed
  
  ADC->CCR = 0;                         // turn tSensor off
  
  DMA_CH1_DISABLE();
  
  ADC1->CHSELR = ADC_VBAT_CHANNEL;      // select AIN1
  
  ADC1->SMPR = (
    1 * ADC_SMPR_SMP_0 |           
    0 * ADC_SMPR_SMP_1 |           
    0 * ADC_SMPR_SMP_2 
  );  

  DMA_CH1_ENABLE(v_buf, ADC_BUF_SIZE);  // configure DMA to get VBAT data

  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    |
#endif
    RCC_AHBENR_SRAMEN     |
    RCC_AHBENR_DMA1EN     |
    RCC_AHBENR_GPIOAEN
  );
  
  START_ADC_CONVERSION();
  __WFE();                            

  DMA_CH1_DISABLE();
  
  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    |
    RCC_AHBENR_SRAMEN     |
    RCC_AHBENR_GPIOAEN    |
#endif
    0
  );
  
  RCC->APB2ENR = 0;
  RCC->CR2 &= ~RCC_CR2_HSI14ON;            // disable HSI14
  
  return true;
}    
