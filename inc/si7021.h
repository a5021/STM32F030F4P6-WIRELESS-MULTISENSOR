#ifndef __SI7021_INCLUDED
#define __SI7021_INCLUDED

#include "i2c.h"

#define SI7021_I2C_ADDRESS              0x40

#define SI7021_MEAS_HUMI_HOLD           0xE5
#define SI7021_MEAS_HUMI_NOHOLD         0xF5
#define SI7021_MEAS_TEMP_HOLD           0xE3
#define SI7021_MEAS_TEMP_NOHOLD         0xF3
#define SI7021_READ_TEMP                0xE0
#define SI7021_RESET                    0xFE
#define SI7021_READ_USER_REG            0xE7
#define SI7021_WRITE_USER_REG           0xE6
#define SI7021_READ_HEATER_CTL_REG      0x11
#define SI7021_WRITE_HEATER_CTL_REG     0x51
#define SI7021_READ_ID_CMD_I            0xFA 
#define SI7021_READ_ID_CMD_II           0x0F
#define SI7021_READ_ID_II_CMD_I         0xFC 
#define SI7021_READ_ID_II_CMD_II        0xC9
#define SI7021_READ_FW_REV_CMD_I        0x84 
#define SI7021_READ_FW_REV_CMD_II       0xB8

__STATIC_INLINE bool si7021_start_conv(uint8_t cmd) {
  I2C_CR1_STOP();
  I2C_START_WRITING(SI7021_I2C_ADDRESS, 1, cmd);
  I2C_SLEEP_UNTIL_STOP();
  return true;
}

__STATIC_INLINE bool si7021_get_result(int32_t *res) {
  uint8_t rData[2];

  I2C_CR1_RX();
  I2C_START_READING(SI7021_I2C_ADDRESS, 2);
  I2C_SLEEP_UNTIL_RXNE();
  rData[0] = I2C1->RXDR;
  I2C_SLEEP_UNTIL_RXNE();
  rData[1] = I2C1->RXDR;
  
  I2C_WAIT_FOR_STOP_FLAG();  // no sleep here

  *res = (rData[0] << 8) | rData[1];

  return true;
}

#endif
