#ifndef __BH1750_INCLUDED
#define __BH1750_INCLUDED

#include "i2c.h"

#define         BH1750_ADDR_PIN_LOW             0
#define         BH1750_ADDR_PIN_HIGH            1
#define         BH1750_ADDRESS_PIN_STATE        BH1750_ADDR_PIN_LOW

////////////////         I2C Slave-address Terminal
////////////////         ADDR = ‘H’ ( ADDR ? 0.7VCC )
////////////////                “1011100“
////////////////         ADDR = 'L' ( ADDR ? 0.3VCC )
////////////////                “0100011“

#define         BH1750_I2C_ADDRESS              (BH1750_ADDRESS_PIN_STATE ? 0x5C : 0x23)

#define         BH1750_POWER_DOWN               0x00
#define         BH1750_POWER_ON                 0x01
#define         BH1750_RESET                    0x07

#define         BH1750_CONTINUOUS_HRES          0x10
#define         BH1750_CONTINUOUS_HRES_II       0x11
#define         BH1750_CONTINUOUS_LRES          0x13
#define         BH1750_ONE_TIME_HRES            0x20
#define         BH1750_ONE_TIME_HRES_II         0x21
#define         BH1750_ONE_TIME_LRES            0x23

#define         BH1750_CHANGE_MES_TIME_HIGH     0x40
#define         BH1750_CHANGE_MES_TIME_LOW      0x60

#define         BH1750_PARAM                    (!SKIP_LUMI(nvStatus)) ? BH1750_MODE(nvStatus) : BH1750_ONE_TIME_HRES_II

__STATIC_INLINE bool bh1750_reset(void) {
  I2C_CR1_STOP();
  I2C_START_WRITING(BH1750_I2C_ADDRESS, 1, BH1750_RESET);
  I2C_SLEEP_UNTIL_STOP();
  return true;
}

__STATIC_INLINE bool bh1750_start_conv(uint8_t resolution) {
  I2C_CR1_STOP();
  I2C_START_WRITING(BH1750_I2C_ADDRESS, 1, resolution);
  I2C_SLEEP_UNTIL_STOP();
  return true;
}  

__STATIC_INLINE bool bh1750_get_result(uint32_t *lx) {
  uint8_t msb;
  I2C_CR1_RX();
  I2C_START_READING(BH1750_I2C_ADDRESS, 2);
  I2C_SLEEP_UNTIL_RXNE();
  msb = (uint8_t) I2C1->RXDR;
  
  I2C_SLEEP_UNTIL_RXNE();
  I2C_WAIT_FOR_STOP_FLAG();

  *lx = (msb << 8) | I2C1->RXDR;
  
  return true;
}  

#endif
