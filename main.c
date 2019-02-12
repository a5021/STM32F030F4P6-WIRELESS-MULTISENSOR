#include <stddef.h>
#include "stm32f0xx.h"
#include "spi.h"
#include "adc.h"
#include "control.h"
#include "bmp180.h"
#include "bh1750.h"
#include "si7021.h"
#include "nrf24l01.h"

// #define SERIAL_DEBUG    1
// #define USE_EXT_CODE 1

#ifdef  SERIAL_DEBUG
  #include "usart.h"
#endif

int main() {

  uint32_t bh1750_lumi,
           wuTime;

  int32_t  si7021_temp,
           si7021_humi,
           bmp180_temp,
           bmp180_press;

  bool     bmp180_ok,
           si7021_ok,
           bh1750_ok,
           adc_ok;

  nvStatus = NV_STATUS; // get status from the backup register (stored before standby)

  SCB->SCR = SCB_SCR_SEVONPEND_Msk;
  __SEV();
  __WFE();

#ifndef SWD_DISABLED
  //Configure_DBG();
#endif

#ifdef  USE_EXT_CODE
  uint32_t csr = RCC->CSR;
#endif

  if (IS_POWER_CYCLE_REQUIRED()) {

    powerCycle();  // make power cycle (off-on) on the vload rail

    nvStatus &= ~(
      NV_NRF_INIT_DONE             |  // request re-init NRF24L01
      NV_BMP180_CAL_DATA_AVAILABLE |  // request retrieval of BMP180 calib. data
      NV_SKIP_ADC                  |  // skip ad conversion
      NV_BH1750_RES_MASK              // set default BH1750 mode
    );

    nvStatus |= (
      NV_POWER_CYCLE_DONE          |  // set power cycle performed flag
      NV_LSI_CALIBRATION_REQ          // set LSI calibration request
    );

    /******************************************************************/
    /*            Wait until sensors become ready                     */
    /******************************************************************/

    S_DELAY(6 * 80);         // (6*80*512*2 + 4 * 512) / 8000 = 61.7ms
  }

  RCC->CSR |= RCC_CSR_RMVF;  // reset flags to prevent unwanted power cycling

  initI2C();                 // init peripherals to handle I2C sensors

  /***************************************************************************/
  /*                           Read sensors                                  */
  /***************************************************************************/

  bh1750_ok = (
    (!SKIP_LUMI(nvStatus) || (PKT_ID(CYCLE_COUNT) == 7)) &&  // check cond.
    bh1750_reset()                                       &&  // reset light sensor
    bh1750_start_conv(BH1750_PARAM)                          // start light conv
  );

  si7021_ok = si7021_start_conv(SI7021_MEAS_HUMI_NOHOLD);    // start humi conv

  bmp180_ok = ((     // read BMP180 sensor
    bmp180_start_conv(BMP180_TEMP)                       &&  // start temp conv
    i2c_sleep(convParam[0].delay)                        &&  // sleep 4.5ms
    bmp180_get_result(&bmp180_temp, BMP180_TEMP)         &&  // get temperature
    bmp180_start_conv(BMP180_PRESS)                      &&  // start press conv
    i2c_sleep(convParam[BMP180_OSS].delay)               &&  // sleep 13.5ms
    bmp180_get_result(&bmp180_press, BMP180_PRESS)       &&  // get pressure
    (IS_BMP180_PROM_AVAILABLE() || bmp180_get_prom()))   ||  // get calib. data
    ! i2c_sleep(148) // (148*2+4)*512/8000=~19.2ms sleep if error
  );

  si7021_ok = ((     // read SI7021 sensor
    si7021_ok                                            &&  // if conv started
    si7021_get_result(&si7021_humi)                      &&  // get humi
    si7021_start_conv(SI7021_READ_TEMP)                  &&  // start conv temp
    si7021_get_result(&si7021_temp)                      &&  // get temperature
    (i2c_disable(), true))                               ||  // disable I2C
    (i2c_disable(), false)                               ||  // disable I2C
    (s_delay(14), false)                                     // sleep if error
  );

  adc_ok = ((        // read vcc, vbat and MCU's temperature sensor
    (!SKIP_ADC(nvStatus) || (PKT_ID(CYCLE_COUNT) == 0))  &&  // check request
    adc_get_data()                                       &&  // do adc measuring
    (nvStatus |= NV_SKIP_ADC))                           ||  // set flag
    (s_delay(6), false)  // 6*512*2/8000 = ~1ms sleep if skipped
  );

  bh1750_ok = (      // read BH1750 sensor
    bh1750_ok                                            &&  // if conv started
    !(IS_BH1750_RES_HIGH() && (s_delay(936), false))     &&  // (936*2+4)*512/8000=~120 ms
    (i2c_enable(), true)                                 &&  // enable i2c
    bh1750_get_result(&bh1750_lumi)                          // get luminosity
  );

  i2c_disable();     // Stop all I2C activity

  /****************************************************************************/
  /*            Configure MCU peripherals to drive NRF24L01                   */
  /****************************************************************************/

  initSPI();                // init SPI peripheral

  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    |
    RCC_AHBENR_SRAMEN     |
#endif
    RCC_AHBENR_GPIOAEN      // enable clock for GPIOA
  );

  GPIOA->ODR = GPIO_ODR_4;              // prepare PA4 (NRF24_CSN) to set HIGH

  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( // Configure GPIOA
#ifndef SWD_DISABLED
    PIN_CONF(PIN(13), PINV_ALT_FUNC) |  // PA13 AF0 -- SYS_SWDIO
    PIN_CONF(PIN(14), PINV_ALT_FUNC) |  // PA14 AF0 -- SYS_SWDCLK
#endif
    PIN_CONF(PIN(3), PINV_OUTPUT)    |  // PA3 OUT  -- NRF24_CE  (set LOW)
    PIN_CONF(PIN(4), PINV_OUTPUT)    |  // PA4 OUT  -- NRF24_CSN (set HIGH)
    PIN_CONF(PIN(5), PINV_ALT_FUNC)  |  // PA5 AF0  -- SPI1_SCK
    PIN_CONF(PIN(6), PINV_ALT_FUNC)  |  // PA6 AF0  -- SPI1_MISO
    PIN_CONF(PIN(7), PINV_ALT_FUNC)     // PA7 AF0  -- SPI1_MOSI
  );

  /****************************************************************************/
  /*            Check if NRF24L01 init sequence needs to be performed         */
  /****************************************************************************/

  if (IS_NRF_INIT_REQUIRED()) {        // check if init required

    nvStatus |= NV_NRF_INIT_DONE;      // set 'init-done' flag

    NRF24_SET_ADDR_WIDTH(NRF24_ADDR_WIDTH_3);
    NRF24_SET_AUTO_ACK(NRF24_AUTO_ACK_DISABLE);
    NRF24_SET_RADIO_CHANNEL(NRF_FREQ_CHANNEL);
    NRF24_SETUP_RADIO(NRF24_TX_POWER_LOWEST, NRF24_DATA_RATE_2M);

  /*

  Page 30 of nRF24L01+ Product Specification:

  In order to enable DPL the EN_DPL bit in the FEATURE register must be enabled.
  In RX mode the DYNPD register must be set. A PTX that transmits to a PRX with
  DPL enabled must have the DPL_P0 bit in DYNPD set.

  */

    NRF24_SET_FEATURE(NRF24_EN_PAYLOAD_NOACK | NRF24_EN_DYN_PAYLOAD);
    NRF24_SET_DPL_PIPE(NRF24_DYNPD_DPL_P0);
    NRF24_FLUSH(TX_FIFO);
  }

  /****************************************************************************/
  /*            Powering NRF24L01's transmitter up                            */
  /****************************************************************************/

  bool nrf_ok = NRF24_POWER_UP();     /* Powering NRF24L01's transmitter up */

  /****************************************************************************/
  /*            Prepare payload while NRF24L01 is warming up                  */
  /****************************************************************************/

  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    |
    RCC_AHBENR_SRAMEN     |
#endif
    0                                        // disable all AHB clocks
  );

  RCC->APB2ENR = 0;                          // disable SPI1

  /************************** Run MCU at 32mHz *******************************/

  turn_pll_on(RCC_CFGR_PLLMUL8);             /* 8 MHz HSI / 2 * 8 = 32 MHz */

  data_pack_t payload;                       // payload packed structure

  payload.pkt_id = PKT_ID(CYCLE_COUNT);      // set data packet ID (4 bits)
  payload.tx_status = TX_STATUS(nvStatus);   // set last TX result bit
  payload.sen_status = (i2c_status == 0);    // set sensors I/O result

  if (bmp180_ok) {                           // if bmp180 data is available
    payload.bmp180_temp = bmp180_calc_rt(bmp180_temp); // convert raw temperature to -512 .. 512
    payload.bmp180_press = KPA_TO_MMHG(bmp180_calc_rp(bmp180_press, BMP180_OSS)) - 700;
  } else {
    payload.bmp180_temp = -512;
  }

  if (si7021_ok) {                           // if si7021 data is available
    si7021_humi = 125 * si7021_humi / 65536 - 6; // convert raw humidity data
    if (si7021_humi > 127) si7021_humi = 127;
    payload.si7021_humi = si7021_humi;       // store it in payload's bits
    si7021_temp = (si7021_temp * 17572 / 65536 - 4685) / 10;
    payload.si7021_temp = si7021_temp;       // store it in payload's bits
  } else {
    si7021_temp = -512;
  }

  void (*turn_regulator)(void) = NULL;       // reset switch func ptr

  if (adc_ok) {                              // if ADC data is available
    uint32_t adc_vcc = VREFINT_CAL * VDD_CALIB / average((uint16_t*)&vref_buf[2]);
    int32_t adc_temp = average(ts_buf) * adc_vcc / VDD_CALIB - TS_CAL_30;
    adc_temp = adc_temp * 80 / (int32_t) (TS_CAL_110 - TS_CAL_30) + 30;
    uint32_t adc_vbat = adc2voltage(v_buf, adc_vcc);  // compute vbat

  /* * * * * * *   Configure wake up time   * * * * * * * * * * * */

#define IGNORE_DAY_AND_HOUR (RTC_ALRMAR_MSK4 | RTC_ALRMAR_MSK3) /* Alarm mask */

    if (adc_vbat > 205) {
      wuTime = IGNORE_DAY_AND_HOUR | (1UL << 8);        // 1 minute
    } else if (adc_vbat > 145) {
      wuTime = IGNORE_DAY_AND_HOUR | (1UL << 8) | 0x30; // 1 minute 30 sec
    } else if (adc_vbat > 115) {
      wuTime = IGNORE_DAY_AND_HOUR | (2UL << 8);        // 2 minutes
    } else if (adc_vbat > 105) {
      wuTime = IGNORE_DAY_AND_HOUR | (2UL << 8) | 0x30; // 2 minutes 30 sec
    } else {
      wuTime = IGNORE_DAY_AND_HOUR | (3UL << 8);        // 3 minutes
    }

  /* * * * * * *   Check the battery state   * * * * * * * * * * * */

    if (adc_vbat == adc_vcc) {               // in case DC-DC is switched off
      if (adc_vbat < 201) {                  // if voltage is below threshold
        turn_regulator = turnRegulatorOn;    // prepare to switch regulator ON
        nvStatus = (nvStatus & ~NV_NRF_INIT_DONE) | NV_LSI_CALIBRATION_REQ;
      }
    } else if (adc_vbat > 226) {             // if voltage is above threshold
      turn_regulator = turnRegulatorOff;     // prepare to switch regulator OFF
      nvStatus = (nvStatus & ~NV_NRF_INIT_DONE) | NV_LSI_CALIBRATION_REQ;
    }

#define P_ARR   ((uint8_t *)(&payload))

    payload.adc_temp = adc_temp;             // put temp value into payload
    payload.adc_vbat = adc_vbat;             // put vbat value into payload
    P_ARR[7] = adc_vcc - 160;                // put vcc value into payload
  } else {
    wuTime = 0;   // reset var to disable unnecessary RTC alarm reprogramming
  }

  if (bh1750_ok) {    // in the case of light sensor data is available

#if defined(__GNUC__) && !defined(__clang__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
    bh1750_lumi = bh1750_lumi * 10 / 12;     // convert raw data to Luxes
#if defined(__GNUC__) && !defined(__clang__)
  #pragma GCC diagnostic pop
#endif

    nvStatus &= ~NV_BH1750_RES_MASK;         // clear BH1750 bits in status var
    if (bh1750_lumi == 0) {                  // in case of illuminance = 0
      nvStatus |= NV_BH1750_DISABLED;        // disable light metering
    } else if (bh1750_lumi < 15) {           // when illumination is weak
      nvStatus |= NV_BH1750_MODE_HIGH_II;    // use HiRes mode 2
    } else if (bh1750_lumi > 200) {          // at high illuminance levels
      nvStatus |= NV_BH1750_MODE_LOW;        // use LowRes mode
    }
    // note: HiRes Mode 1 (both bits are low) will be set if light
    //       intensity level is in range from 15 to 199.
  }

    // calculate payload size that depends on the data available
  uint8_t pSize = (NRF24_PAYLOAD_SIZE / 2) +
                  ((SKIP_LUMI(nvStatus)) ? 0 : 2) +
                  ((PKT_ID(CYCLE_COUNT) == 0) ? 3 : 0);

  if (pSize == 7) {

    #if defined(__GNUC__) && !defined(__clang__)
      #pragma GCC diagnostic push
      #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
    #endif

    P_ARR[5] = ((uint8_t *)(&bh1750_lumi))[0];
    P_ARR[6] = ((uint8_t *)(&bh1750_lumi))[1];

    #if defined(__GNUC__) && !defined(__clang__)
      #pragma GCC diagnostic pop
    #endif

  } else if (pSize == 10) {
    ((uint16_t *)(&payload))[4] = bh1750_lumi; // payload.bh1750 = bh1750_lumi;
  }

#ifdef USE_EXT_CODE
  csr = (uint8_t)((csr >> 24) | (((csr & RCC_CSR_V18PWRRSTF) == RCC_CSR_V18PWRRSTF) ? 0 : 1);
  if (csr != 0) {
    P_ARR[10] = csr;  //payload.ext_code = csr;
  }
#endif

  if ((CYCLE_COUNT & 64UL) == 64UL) {             // every hour
    nvStatus &= ~NV_EVENT_HOURLY;
    if ((CYCLE_COUNT & 128UL) == 128UL) {         // every two hours
      nvStatus &= ~NV_NRF_INIT_DONE;
      if ((CYCLE_COUNT & 256UL) == 256UL) {       // every four hours
        nvStatus |= NV_LSI_CALIBRATION_REQ;
      }
    }
    if ((CYCLE_COUNT & 2048UL) == 2048UL) {       // every 34 hours
      nvStatus &= ~NV_EVENT_DAILY;
      if ((CYCLE_COUNT & 8192UL) == 8192UL) {     // every 5.6 days
        nvStatus &= ~NV_EVENT_WEEKLY;
        if ((CYCLE_COUNT & 32768UL) == 32768UL) { // every 22.7 days
          nvStatus &= ~NV_EVENT_MONTHLY;
        }
      }
    }
  }

  /************************** Run MCU at 8mHz ********************************/

  turn_pll_off(); /* restore the default MCU speed */

    RCC->AHBENR = (
#ifndef SWD_DISABLED
      RCC_AHBENR_FLITFEN  |
      RCC_AHBENR_SRAMEN   |
#endif
      RCC_AHBENR_GPIOAEN                          // enable clock for GPIOA
    );
  RCC->APB2ENR = RCC_APB2ENR_SPI1EN;              // enable clock for SPI1

  /****************************************************************************/
  /*            Transfer payload to NRF24L01's TX FIFO                        */
  /****************************************************************************/

  NRF24_WRITE_PAYLOAD(payload, pSize);

  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    |
    RCC_AHBENR_GPIOAEN    |    // enable clock for GPIOA in debug mode
    RCC_AHBENR_SRAMEN     |
#endif
    0                          // disable clock for GPIOA
  );

  RCC->APB2ENR = 0;            // disable clock for SPI1

  /****************************************************************************/
  /*          Tpd2stby delay (wait till NRF24L01's oscillator becomes stable) */
  /****************************************************************************/
  /*                                                                          */
  /*  The delay time depends on crystall's equivalent serial inductance (Ls)  */
  /*            Tpd2stby = Ls / 30mH * 1.5ms if Ls exceeds 30mH               */
  /*                                                                          */
  /*    Typical delay value is                                                */
  /*            - 1.5ms if Ls < 30mH;                                         */
  /*            - 3ms if Ls = 60mH;                                           */
  /*            - 4.5ms if Ls = 90mH.                                         */
  /*                                                                          */
  /****************************************************************************/

  S_DELAY(11);                 // (11*2 + 4) * 512 / 8000 = 1.664ms

  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    |
    RCC_AHBENR_SRAMEN     |
#endif
    RCC_AHBENR_GPIOAEN    |    // enable clock for GPIOA
    RCC_AHBENR_GPIOFEN         // enable clock for GPIOF
  );

  /****************************************************************************/
  /*            Start transmission                                            */
  /****************************************************************************/

  CE(HIGH);                                    // start TX pulse

  /****************************************************************************/
  /*            Configure PF1 as external interrupt line (NRF24L01 IRQ)       */
  /****************************************************************************/

  GPIOF->MODER = ANALOG_MODE_FOR_ALL_PINS - PIN_CONF(PIN(1), PINV_INPUT);
  RCC->APB2ENR = RCC_APB2ENR_SYSCFGCOMPEN;     // enable clock for SYSCFG

  SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PF; // set PF1 as EXTI line

  RCC->APB2ENR = 0;                            // disable clock for SYSCFG

  EXTI->EMR  = 0x0002;                         // enable event
  EXTI->FTSR = 0x0002;                         // on falling edge

  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    |
    RCC_AHBENR_SRAMEN     |
    RCC_AHBENR_GPIOAEN    |    // enable clock for GPIOA in debug mode
#endif
    RCC_AHBENR_GPIOFEN
  );

  CE(LOW);                                     // finish TX pulse

  /****************************************************************************/
  /*            Sleep till transmission ends                                  */
  /****************************************************************************/

  if (nrf_ok) {
    RUN_MCU_AT(HF_MHZ);        // slow down CPU
    __WFE();                   // sleep till NRF's IRQ appears
  } else {
    S_DELAY(2);                // (2*2 + 4) * 512 / 8000 = 512us
  }

  RUN_MCU_AT(EIGHT_MHZ);

  /****************************************************************************/
  /*            Switch NRF24L01 to Power Down mode                            */
  /****************************************************************************/

  RCC->APB2ENR = RCC_APB2ENR_SPI1EN;        // disable SYSCFG, enable SPI1

  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    |
    RCC_AHBENR_SRAMEN     |
#endif
    RCC_AHBENR_GPIOAEN         // enable clock for GPIOA
  );

  NRF24_POWER_DOWN();          // Stop oscillator and go into Power Down Mode

  /************ Reset NRF24L01's status bits and release IRQ line *************/

  nrf_ok = (NRF24_TX_OK == NRF24_RESET_STATUS()) && nrf_ok;

  RCC->APB2ENR = 0;            // turn off SPI

  /****************************************************************************/
  /*            Turn DC-DC ON/OFF if required                                 */
  /****************************************************************************/

  if (turn_regulator != NULL) {
    turn_regulator();
  }

  RCC->AHBENR = (
#ifndef SWD_DISABLED
    RCC_AHBENR_FLITFEN    |
    RCC_AHBENR_SRAMEN     |
    RCC_AHBENR_GPIOAEN    |
#endif
    0                          // Disable all AHB peripherials
  );

  /****************************************************************************/

  if (nrf_ok) {
    nvStatus |= NV_LAST_TX_ATTEMPT_OK;
  } else {
    nvStatus &= ~(NV_LAST_TX_ATTEMPT_OK | NV_NRF_INIT_DONE) ;
      // check if at least two transmissions consequently failed
    if (TX_STATUS(nvStatus) == 0) {
      if ((nvStatus & NV_EVENT_DAILY) == 0) {
        nvStatus = (nvStatus & ~NV_POWER_CYCLE_DONE) | NV_EVENT_DAILY;
      }
    }
  }

  /****************************************************************************/
  /*            Prepare to go into standby mode                               */
  /****************************************************************************/

  if (((RTC->ISR & RTC_ISR_INITS) != RTC_ISR_INITS) ||
     ((nvStatus & NV_LSI_CALIBRATION_REQ) == NV_LSI_CALIBRATION_REQ)) {

    if ((RCC->CSR & RCC_CSR_LSIRDY) != RCC_CSR_LSIRDY) {
      RCC->CSR |= RCC_CSR_LSION;         // enable LSI
      while((RCC->CSR & RCC_CSR_LSIRDY) != RCC_CSR_LSIRDY);  // wait till LSI becomes stable
    }

    uint32_t prep = calc_rtc_divider();  // recalc RTC prescalers
    nvStatus &= ~NV_LSI_CALIBRATION_REQ; // reset RTC calibration request

    RCC->APB1ENR = RCC_APB1ENR_PWREN;    // enable power interface clock
    PWR->CR = PWR_CR_DBP;                // enable write to backup domain

    RCC->BDCR = (                        // with backup domain control reg ...
      RCC_BDCR_RTCEN      |              // enable RTC
      RCC_BDCR_RTCSEL_1                  // set LSI as RTC clock source
    );

    RTC_WRITE_ENABLE();                  // disable RTC write protection

    RTC->ISR |= RTC_ISR_INIT;            // enter into RTC init stage
    while ((RTC->ISR & RTC_ISR_INITF) != RTC_ISR_INITF); // wait RTC is stopped

    RTC->PRER = prep;                    // set sync and async RTC prescaler

#ifdef __SES_ARM
  #define RTC_DR_YT_Pos                  (20U)
  #define RTC_DR_YU_Pos                  (16U)
#endif  

    RTC->DR = (1 << RTC_DR_YT_Pos) | (9 << RTC_DR_YU_Pos); // YEAR = 19

  } else {
    RCC->APB1ENR = RCC_APB1ENR_PWREN;    // enable power interface clock
    PWR->CR = PWR_CR_DBP;                // enable write to backup domain

    RTC_WRITE_ENABLE();                  // disable RTC write protection

    RTC->ISR |= RTC_ISR_INIT;            // enter into RTC init stage
    while ((RTC->ISR & RTC_ISR_INITF) != RTC_ISR_INITF); // wait RTC is stopped
  }

  RTC->TR = (uint32_t) 0;                // reset time to zero

  if ((wuTime != 0) && (wuTime != RTC->ALRMAR)) { // if a new wake up time is available
    RTC->CR &=~ RTC_CR_ALRAE;            // enter into alarm init stage
    while ((RTC->ISR & RTC_ISR_ALRAWF) != RTC_ISR_ALRAWF); // wait till alarm becomes inactive
    RTC->ALRMAR = wuTime;                // program alarm with new settings
    RTC->CR = (
      RTC_CR_ALRAE        |              // alarm enable
      RTC_CR_ALRAIE       |              // alarm interrupt enable
      RTC_CR_BYPSHAD                     // bypass RTC shadow registers
    );
  }

  CYCLE_COUNT++;                         // update cycle counter
  NV_STATUS = nvStatus;                  // save status into backup register

  RTC->ISR &= ~(RTC_ISR_ALRAF | RTC_ISR_INIT); // exit RTC init stage

  RTC_WRITE_DISABLE();                   // protect RTC registers for writing

  //     Standby mode entry:
  //     ===================
  //     WFI (Wait for Interrupt) or WFE (Wait for Event) while:
  //     – Set SLEEPDEEP bit in System Control register (SCB_SCR)
  //     – Set PDDS bit in Power Control register (PWR_CR)
  //     – Clear WUF bit in Power Control/Status register (PWR_CSR)

  PWR->CR = (
    PWR_CR_PDDS           |              // set PPDS bit (1 = Standby, 0 = Stop)
    PWR_CR_CSBF           |              // clear Standby flag
    PWR_CR_CWUF                          // clear wakeup flag
  );

  SCB->SCR = SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SEVONPEND_Msk;

  __WFE();                               // enter standby

  /*************** !!! Should never get here !!! ******************************/

  NVIC_SystemReset();                    // Invoke System Reset
}

/*** END OF PROGRAM  ***/
