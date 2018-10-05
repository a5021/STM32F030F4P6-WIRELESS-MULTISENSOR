#ifndef __BMP180_INCLUDED
#define __BMP180_INCLUDED

#include <stdbool.h>
#include "i2c.h"
#include "control.h"
#include "flash.h"

#define BMP180_I2C_ADDRESS              0x77 // 7-bit address

#define	BMP180_REG_CONTROL              0xF4
#define	BMP180_REG_RESULT               0xF6

#define	BMP180_COMMAND_TEMPERATURE      0x2E
#define	BMP180_COMMAND_PRESSURE0        0x34
#define	BMP180_COMMAND_PRESSURE1        0x74
#define	BMP180_COMMAND_PRESSURE2        0xB4
#define	BMP180_COMMAND_PRESSURE3        0xF4

        // Registers
#define BMP180_PROM_START_ADDR          0xAA // E2PROM calibration data start register
#define BMP180_PROM_DATA_LEN            22   // E2PROM length
#define BMP180_CHIP_ID_REG              0xD0 // Chip ID
#define BMP180_VERSION_REG              0xD1 // Version
#define BMP180_CTRL_MEAS_REG            0xF4 // Measurements control (OSS[7.6], SCO[5], CTL[4.0]
#define BMP180_ADC_OUT_MSB_REG          0xF6 // ADC out MSB  [7:0]
#define BMP180_ADC_OUT_LSB_REG          0xF7 // ADC out LSB  [7:0]
#define BMP180_ADC_OUT_XLSB_REG         0xF8 // ADC out XLSB [7:3]
#define BMP180_SOFT_RESET_REG           0xE0 // Soft reset control
        // Commands
#define BMP180_T_MEASURE                0x2E // temperature measurement
#define BMP180_P0_MEASURE               0x34 // pressure measurement (OSS=0, 4.5ms)
#define BMP180_P1_MEASURE               0x74 // pressure measurement (OSS=1, 7.5ms)
#define BMP180_P2_MEASURE               0xB4 // pressure measurement (OSS=2, 13.5ms)
#define BMP180_P3_MEASURE               0xF4 // pressure measurement (OSS=3, 25.5ms)
        // Constants
#define BMP180_PARAM_MG                 ((int32_t)3038)
#define BMP180_PARAM_MH                 ((int32_t)-7357)
#define BMP180_PARAM_MI                 ((int32_t)3791)

#define BMP180_START_OF_CONVERSION_BIT  (1 << 5)

#define BMP180_TEMP                     2
#define BMP180_PRESS                    3

        // Calibration parameters structure
typedef struct {
          uint16_t  PAD;    // * two padding bytes (not available in bmp180 PROM)
          int16_t   AC1;
          int16_t   AC2;
          int16_t   AC3;
          uint16_t  AC4;
          uint16_t  AC5;
          uint16_t  AC6;
          int16_t   B1;
          int16_t   B2;
          int16_t   MB;
          int16_t   MC;
          int16_t   MD;
          uint32_t  CRC32;  // * checksum (not available in bmp180 PROM)
} BMP180_Calibration_TypeDef;

typedef struct {
          uint8_t cmd;
          uint8_t delay;
} BMP180_conv_param_t;

extern const BMP180_conv_param_t convParam[];
extern int32_t B5;

#define CAL_DATA ((BMP180_Calibration_TypeDef *) FLASH_DATA_ADDR)

#define KPA_TO_MMHG(PRESSURE) ((uint32_t)(PRESSURE * 75) / 10000)

__STATIC_INLINE int32_t bmp180_calc_rt(int32_t ut) {
  int32_t X1 = (ut - CAL_DATA->AC6) * CAL_DATA->AC5 / 32768;
  int32_t X2 = CAL_DATA->MC * 2048 / (X1 + CAL_DATA->MD);
  B5 = X1 + X2;
  return  (B5 + 8) / 16;
}  

__STATIC_INLINE int32_t bmp180_calc_rp(int32_t up, uint8_t oss){
  int32_t B3, B6, X1, X2, X3, p;
  uint32_t B4, B7;
  
  B6 = B5 - 4000;
  X1 = (CAL_DATA->B2 * (B6 * B6 / 4096)) / 2048;
  X2 = CAL_DATA->AC2 * B6 / 2048;
  X3 = X1 + X2;
  B3 = ((((int32_t)CAL_DATA->AC1 * 4 + X3) << oss) + 2) / 4;
  X1 = CAL_DATA->AC3 * B6 / 8192;
  X2 = (CAL_DATA->B1 * (B6 * B6 / 4096)) / 65536;
  X3 = ((X1 + X2) + 2) / 4;
  B4 = CAL_DATA->AC4 * (uint32_t) (X3 + 32768) / 32768;
  up >>= (8 - oss);
  B7 = ((uint32_t) up - B3) * (50000 >> oss);
  
  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  
  X1 = (p / 256) * (p / 256);
  X1 = (X1 * 3038) / 65536;
  X2 = (-7357 * p) / 65536;
  p = p + (X1 + X2 + 3791) / 16;
  return p;
}

__STATIC_INLINE bool bmp180_start_conv(uint8_t conv_type) {
  I2C_CR1_TX();
  I2C_START_WRITING(BMP180_I2C_ADDRESS, 2, BMP180_CTRL_MEAS_REG);
  I2C_SLEEP_UNTIL_TXE();
  I2C1->TXDR = (conv_type == BMP180_TEMP) ? BMP180_T_MEASURE : convParam[BMP180_OSS].cmd;
  I2C_WAIT_FOR_STOP_FLAG();
  return true;
}

__STATIC_INLINE bool bmp180_get_result(int32_t *r, uint8_t r_type) {
  // uint8_t lsb, msb;
  
  I2C_CR1_STOP();
  I2C_START_WRITING(BMP180_I2C_ADDRESS, 1, BMP180_ADC_OUT_MSB_REG);
  I2C_SLEEP_UNTIL_STOP();

  I2C_CR1_RX();
  I2C_START_READING(BMP180_I2C_ADDRESS, r_type);
  I2C_SLEEP_UNTIL_RXNE();
  uint8_t msb = I2C1->RXDR;
  I2C_SLEEP_UNTIL_RXNE();
  if (r_type == BMP180_TEMP) {
    *r = ((msb << 8) | I2C1->RXDR);
  } else {
    uint8_t lsb = I2C1->RXDR;
    I2C_SLEEP_UNTIL_RXNE();
    *r = ((msb << 16) | (lsb << 8) | I2C1->RXDR);
  }
  I2C_WAIT_FOR_STOP_FLAG();
  return true;
}

bool bmp180_get_prom(void);

#endif
