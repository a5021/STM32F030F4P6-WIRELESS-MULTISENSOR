#ifndef __CONTROL_INCLUDED
#define __CONTROL_INCLUDED

#define NRF_FREQ_CHANNEL        99
#define BMP180_OSS              2

//#define USE_EXT_CODE 1

#define EIGHT_MHZ               RCC_CFGR_HPRE_DIV1
#define FOUR_MHZ                RCC_CFGR_HPRE_DIV2
#define TWO_MHZ                 RCC_CFGR_HPRE_DIV4
#define ONE_MHZ                 RCC_CFGR_HPRE_DIV8
#define HF_MHZ                  RCC_CFGR_HPRE_DIV16
#define LOWEST_FREQ             RCC_CFGR_HPRE_DIV512

#define RUN_MCU_AT(SPEED)       RCC->CFGR = (SPEED)

    // Disable the write protection for RTC registers.
#define RTC_WRITE_ENABLE()      RTC->WPR = 0xCA; RTC->WPR = 0x53

  // Enable the write protection for RTC registers.
#define RTC_WRITE_DISABLE()     RTC->WPR = 0xFF

#define NV_NRF_INIT_DONE        (1U << 0)
#define NV_BMP180_CAL_DATA_AVAILABLE (1U << 1)
#define NV_LAST_TX_ATTEMPT_OK   (1U << 2)
#define NV_POWER_CYCLE_DONE     (1U << 3)

#define NV_SKIP_ADC             (1U << 4)
#define NV_LSI_CALIBRATION_REQ  (1U << 5)

#define NV_BH1750_RES_0         (1U << 6)
#define NV_BH1750_RES_1         (1U << 7)
#define NV_BH1750_RES_MASK      (NV_BH1750_RES_0 + NV_BH1750_RES_1)

#define NV_EVENT_HOURLY         (1U << 8)
#define NV_EVENT_DAILY          (1U << 9)
#define NV_EVENT_WEEKLY         (1U << 10)
#define NV_EVENT_MONTHLY        (1U << 11)
#define NV_EVENT_MASK           (NV_EVENT_HOURLY | NV_EVENT_DAILY | NV_EVENT_WEEKLY | NV_EVENT_MONTHLY)

           // HIGH RES MODE 1  ==  0                                     [0x20]
           // HIGH RES MODE 2  ==  1 (NV_BH1750_RES_0)                   [0x21]
           // PROBE DISABLED   ==  2 (NV_BH1750_RES_1)
           // LOW RES          ==  3 (NV_BH1750_RES_0 + NV_BH1750_RES_1) [0x23]

#define NV_BH1750_DISABLED      NV_BH1750_RES_1
#define NV_BH1750_MODE_HIGH     0
#define NV_BH1750_MODE_HIGH_II  NV_BH1750_RES_0
#define NV_BH1750_MODE_LOW      NV_BH1750_RES_MASK

#define CYCLE_COUNT             RTC->BKP3R
#define NV_STATUS               RTC->BKP4R

#define PKT_ID(VAR)             ((uint8_t) (VAR & 0x07))
#define TX_STATUS(VAR)          ((uint8_t) ((VAR & NV_LAST_TX_ATTEMPT_OK) == NV_LAST_TX_ATTEMPT_OK))
#define SKIP_LUMI(VAR)          ((uint8_t) ((VAR & NV_BH1750_RES_MASK) == NV_BH1750_RES_1))
#define BH1750_MODE(VAR)        (((VAR & NV_BH1750_RES_MASK) >> 6) + 0x20)
#define SKIP_ADC(VAR)           ((uint8_t) ((VAR & NV_SKIP_ADC) == NV_SKIP_ADC))

#define IS_NRF_INIT_REQUIRED()         ((NV_STATUS & NV_NRF_INIT_DONE) == 0)
#define IS_BMP180_PROM_AVAILABLE()     ((NV_STATUS & NV_BMP180_CAL_DATA_AVAILABLE) != 0)
#define IS_LAST_TX_FAILED()            ((NV_STATUS & NV_LAST_TX_ATTEMPT_OK) != NV_LAST_TX_ATTEMPT_OK)

#define IS_POWER_CYCLE_REQUIRED()                                         \
          ((PWR->CSR & PWR_CSR_SBF) != PWR_CSR_SBF)               ||      \
          ((RCC->CSR & (RCC_CSR_PINRSTF | RCC_CSR_SFTRSTF)) != 0) ||      \
          ((nvStatus & NV_POWER_CYCLE_DONE) == 0)

#define IS_BH1750_RES_HIGH()    ((nvStatus & NV_BH1750_RES_MASK) != NV_BH1750_MODE_LOW)

#ifdef __ICCARM__
typedef __packed struct {
#elif defined (__GNUC__)
typedef struct {
#elif defined (__CC_ARM)
typedef struct {
#endif

  uint32_t tx_status     :1;  //  1
  uint32_t pkt_id        :4;  //  1
  uint32_t sen_status    :1;  //  1
  int32_t  bmp180_temp   :10; //  2 [2 bytes]
  uint32_t bmp180_press  :7;  //  1
  uint32_t si7021_humi   :7;  //  2
  int32_t  si7021_temp   :10; //  2 [3 bytes]
  // ------------------------------ 5 bytes boundary
  int32_t  adc_temp      :7;  //  1
  uint32_t adc_vbat      :9;  //  1 [2 bytes]
  uint32_t adc_vcc       :8;  //  1
  uint32_t bh1750        :16; //  3
#ifdef USE_EXT_CODE
  uint32_t ext_code      :8;  //  3
  uint32_t aux           :8;  //  3
#endif

#ifdef __ICCARM__
} data_pack_t;
#elif defined (__GNUC__)
} __attribute__((packed)) data_pack_t;
#elif defined (__CC_ARM)
} __attribute__((packed)) data_pack_t;
#endif

extern uint32_t i2c_status;
extern uint32_t nvStatus;
extern uint16_t flash_status;

__STATIC_INLINE uint32_t calc_rtc_divider(void) {

  uint16_t preDiv_S;

#define PREDIV_A ((uint16_t) 128)

  uint16_t skipCnt = 8;
  uint32_t sSum = 0, prev_pulse = 0;
  uint8_t pCnt = 0;

#define SAMPLE_NUM 16UL

  RCC->APB1RSTR = RCC_APB1RSTR_TIM14RST;
  __NOP(); __NOP(); __NOP();
  RCC->APB1RSTR = 0;
  RCC->APB1ENR = RCC_APB1ENR_TIM14EN;

  RCC->CFGR |= RCC_CFGR_MCO_LSI;          // set LSI as MCO source

  TIM14->CCMR1 |= TIM_CCMR1_CC1S_0;
  TIM14->OR = TIM14_OR_TI1_RMP;
  TIM14->CCER |= TIM_CCER_CC1E;
  TIM14->CR1 = TIM_CR1_URS;
  TIM14->EGR = TIM_EGR_UG;    // Update Generation
  TIM14->CR1 = TIM_CR1_CEN;   // run timer

  while ((TIM14->CR1 & TIM_CR1_CEN) == TIM_CR1_CEN){
    if ((TIM14->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF) {
      if ((TIM14->SR & TIM_SR_CC1OF) != TIM_SR_CC1OF) {
        if (skipCnt == 0) {
          sSum += TIM14->CCR1 - prev_pulse;
          if (++pCnt == SAMPLE_NUM) {
            TIM14->CCER &= ~TIM_CCER_CC1E;
            TIM14->CR1 &= ~TIM_CR1_CEN;
          }
        } else {
          skipCnt--;
        }
        prev_pulse = TIM14->CCR1;
      } else {
        pCnt = 0;
        sSum = 0;
        TIM14->SR &= ~TIM_SR_CC1OF;
      }
      TIM14->SR &= ~TIM_SR_CC1IF;
    }
  };

  RCC->APB1ENR = 0;                       // Disable TIM14
  RCC->CFGR = 0;                          // Disable LSI as MCO source
  RCC->APB1RSTR = RCC_APB1RSTR_TIM14RST;  // Reset TIM14

  preDiv_S = 8000000UL * SAMPLE_NUM / ((sSum == 0) ? 3040UL : sSum) / PREDIV_A;

  RCC->APB1RSTR = 0;                      // Reset done

  return (uint32_t) (((PREDIV_A - 1) << 16) | (preDiv_S + 1));
}

__STATIC_INLINE uint32_t crc32_prom(uint32_t s[]) {

  RCC->AHBENR |= RCC_AHBENR_CRCEN;

  CRC->CR = CRC_CR_RESET;

  CRC->DR = s[0];
  CRC->DR = s[1];
  CRC->DR = s[2];
  CRC->DR = s[3];
  CRC->DR = s[4];
  CRC->DR = s[5];

  uint32_t crc = CRC->DR;
  RCC->AHBENR &= ~RCC_AHBENR_CRCEN;

  return crc;
}

__STATIC_INLINE void turn_pll_on(uint32_t mul) {
  RCC->CFGR = mul;                             // Set the PLL multiplier

  // FLASH->ACR |= (FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE);
  RCC->CR |= RCC_CR_PLLON;                     // Enable the PLL
  while((RCC->CR & RCC_CR_PLLRDY) == 0);       // Wait until PLLRDY is set
  RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL);   // Select PLL as system clock
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait until the PLL is switched on
}

__STATIC_INLINE void turn_pll_off(void) {
  RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW);
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
  RCC->CR &= (uint32_t)(~RCC_CR_PLLON);
  while((RCC->CR & RCC_CR_PLLRDY) != 0);
  // FLASH->ACR &= ~(FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE);
}

void turnRegulatorOn(void);
void turnRegulatorOff(void);
void powerCycle(void);

#ifndef SWD_DISABLED
__STATIC_INLINE void Configure_DBG(void) {
  /* Enable the peripheral clock of DBG register */
  RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;

  DBGMCU->CR |= DBGMCU_CR_DBG_STANDBY; // enable debug in standby mode
}
#endif

#endif
