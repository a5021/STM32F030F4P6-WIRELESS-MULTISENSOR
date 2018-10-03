#ifndef __ADC_INCLUDED
#define __ADC_INCLUDED

#include <stdbool.h>

#define TS_CAL_30          (*(uint16_t*) (0x1ffff7b8))
#define TS_CAL_110         (*(uint16_t*) (0x1FFFF7C2))
#define VREFINT_CAL        (*(uint16_t*) (0x1FFFF7BA))
#define VDD_CALIB          ((uint16_t)   (330))

#define ADC_BUF_SIZE       16

extern uint16_t vref_buf[], ts_buf[], v_buf[];

__STATIC_INLINE uint32_t average(uint16_t b[]) {
  uint32_t s = 0;

  for (uint32_t i = 0; i < ADC_BUF_SIZE; i++) {
    s += b[i];
  }
  return s / ADC_BUF_SIZE;
}

__STATIC_INLINE uint32_t adc2voltage(uint16_t buf[], uint32_t v_ref) {

  uint32_t avr = average(buf);    /* average adc data             */

  if (4095UL == avr) {            /* check if it is equal to VREF */
    return v_ref;                 /* return VREF value            */
  }                               /* recalc v_bat value           */
  avr *= v_ref;                   /* v = adc * V_REF / 4095       */
  avr /= 4095UL;

  return avr;
}

uint32_t adc_read_vload(void);
bool adc_get_data(void);

#endif
