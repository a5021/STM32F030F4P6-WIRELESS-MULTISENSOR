# STM32F030F4P6-WIRELESS-MULTISENSOR

[![Build](https://github.com/a5021/STM32F030F4P6-WIRELESS-MULTISENSOR/actions/workflows/build.yml/badge.svg)](https://github.com/a5021/STM32F030F4P6-WIRELESS-MULTISENSOR/actions/workflows/build.yml) [![MCU](https://img.shields.io/badge/MCU-STM32F030F4P6-00A9E0)]() [![Radio](https://img.shields.io/badge/Radio-nRF24L01-00A9E0)]() [![Sensors](https://img.shields.io/badge/Sensors-BMP180_SI7021_BH1750-00A9E0)]() [![IDE](https://img.shields.io/badge/IDE-Make%20%7C%20EWARM%20%7C%20MDK--ARM%20%7C%20SES-00A9E0)]()

Multi-sensor wireless node based on STM32F030F4P6. Reads environmental data from BMP180 (pressure, temperature), SI7021 (humidity, temperature), and BH1750 (illuminance), monitors battery and die temperature via ADC, and transmits a composite packet over an nRF24L01+ radio link. Designed for ultra-low-power battery operation with adaptive duty cycling.

## Features

- Register-level, DMA-driven peripheral drivers (I2C, SPI, ADC, RTC)
- Three environmental sensors on shared I2C bus:
  - Bosch BMP180 - pressure and temperature (int32 compensation)
  - Silicon Labs SI7021 - humidity and temperature
  - Rohm BH1750 - illuminance (dynamic resolution selection)
- nRF24L01+ radio in Enhanced ShockBurst mode, variable payload (7-11 bytes)
- ADC VREFINT, VBAT, VLOAD monitoring with DMA and auto-off
- Adaptive wake interval: 1-3 minutes based on VBAT level
- Power cycle of sensor rail on cold boot (load switch PF0 + DC-DC PA0)
- CRC32-verified BMP180 calibration data stored in FLASH page
- Standby mode between cycles (RTC alarm wake)
- LSI calibration routine for RTC timing accuracy
- Hourly / daily / weekly / monthly maintenance events

## Hardware Specification

| Component | Detail |
|-----------|--------|
| MCU | STMicroelectronics STM32F030F4P6 (ARM Cortex-M0, 16 KB Flash, 4 KB RAM) |
| Radio | Nordic nRF24L01+ (SPI1, 2 Mbps, channel 99, lowest TX power) |
| Pressure | Bosch BMP180 (I2C addr 0x77, OSS up to ultra-high-res) |
| Humidity | Silicon Labs SI7021 (I2C addr 0x40) |
| Light | Rohm BH1750FVI (I2C addr 0x23, 0.5-65535 lx) |
| Power | DC-DC converter (PA0), load switch (PF0), VDD monitoring via PVD |
| Debug | SWD on PA13/PA14 |

## Pin Assignment

| Signal | Pin  | Peripheral    | Notes                         |
|--------|------|---------------|-------------------------------|
| I2C_SCL| PA9  | I2C1          | Shared bus, 100 kHz           |
| I2C_SDA| PA10 | I2C1          |                               |
| SPI_SCK| PA5  | SPI1          | nRF24L01+, 8 MHz              |
| SPI_MISO| PA6 | SPI1          |                               |
| SPI_MOSI| PA7 | SPI1          |                               |
| NRF_CE | PA3  | GPIO output    | Chip enable, active high      |
| NRF_CSN| PA4  | GPIO output    | SPI chip select, active low   |
| NRF_IRQ| PF1  | EXTI1 (FI)    | Falling edge, wake from sleep |
| DC_SW  | PA0  | GPIO output    | DC-DC converter control       |
| LOAD_SW| PF0  | GPIO output    | Sensor rail load switch       |
| VLOAD_MON| PB1 | AIN9 (ADC)   | Sensor rail voltage monitoring|
| VBAT_MON| PA1 | AIN1 (ADC)    | Battery voltage via divider   |

## Radio Protocol - Packet Format

Variable-length payload transmitted via nRF24L01+ in Enhanced ShockBurst mode (NoACK, dynamic payload, 3-byte address, channel 99):

| Offset | Size    | Field           | Description                                         |
|--------|---------|-----------------|-----------------------------------------------------|
| 0      | 4 bits  | Packet ID       | Rolling 4-bit identifier (derived from cycle count) |
| 0.5    | 1 bit   | TX Status       | Last transmission result (0 = fail, 1 = success)    |
| 0.6    | 1 bit   | Sensor Status   | I2C sensors I/O result (0 = error, 1 = success)     |
| 1      | 6 bits  | BMP180 Temp     | Compensated temperature, signed, -512..512 scale     |
| 1.6    | 10 bits | BMP180 Pressure | Compensated pressure, mmHg - 700, unsigned          |
| 3      | 7 bits  | SI7021 Humidity | Max 127 %RH                                         |
| 3.7    | 9 bits  | SI7021 Temp     | Temperature, signed, C x 10                        |
| 5      | 2 bytes | ADC Temp        | MCU die temperature, C x 1                         |
| 7      | 2 bytes | VBAT            | Battery voltage, mV                                  |
| 9      | 1 byte  | VCC / Ext       | VREFINT-calibrated VDD or extended code             |
| 10     | 2 bytes | BH1750 Light    | Illuminance, lux x 10/12 (present in every 8th pkt) |

Total payload size varies: 7 B (light skipped), 10 B (full), 11 B (with ext code).

## Firmware Architecture

```
  Cold boot
     |
  +-----------+     +----------+     +-----------+     +------------+
  | powerCycle|---->| initI2C  |---->| BMP180    |---->| SI7021     |
  | (rail ON) |     | (400 kHz)|     | T+P conv  |     | H+T conv   |
  +-----------+     +----------+     +-----------+     +------------+
                                           |                  |
                                     +-----v------------------v----+
                                     |        ADC (DMA)            |
                                     |  VREFINT | VBAT | TSEN      |
                                     +-----v------------------v----+
                                           |                  |
                                     +-----v------------------v----+
                                     |      BH1750 (if not skipped)|
                                     +-----v------------------v----+
                                           |
                                     +-----v------------------v----+
                                     |  nRF24L01+ TX              |
                                     |  Payload -> FIFO -> TX     |
                                     |  Sleep till IRQ (WFE)      |
                                     +-----v------------------v----+
                                           |
                                     +-----v------------------v----+
                                     |  RTC Alarm Set              |
                                     |  (adaptive interval)        |
                                     |  Enter STANDBY (WFE)        |
                                     +-----------------------------+
                                           |
                                      (RTC alarm)
                                           |
                                        Wake -> main()
```

## Power Management

| State               | Current     | Notes                             |
|---------------------|-------------|-----------------------------------|
| Active (sensors)    | ~5-15 mA    | I2C + ADC + SPI active            |
| Radio TX            | ~13 mA      | nRF24L01+ TX burst, ~1.5 ms       |
| Standby (RTC alarm) | ~2.5 uA     | Cortex-M0 STOP + RTC on LSI       |
| Sensor rail off     | ~1.5 uA     | Load switch PF0 disables sensors  |

Wake interval is adaptive based on battery voltage:

| VBAT (divider) | Interval |
|----------------|----------|
| >2.05 V        | 1 min    |
| 1.45-2.05 V    | 1.5 min  |
| 1.15-1.45 V    | 2 min    |
| 1.05-1.15 V    | 2.5 min  |
| <1.05 V        | 3 min    |

If VBAT drops below 2.01 V (divider reading), the DC-DC converter is enabled. Above 2.26 V, it is disabled.

## Getting Started

### Prerequisites

- ARM GCC toolchain (arm-none-eabi-gcc)
- GNU Make

### Build & Flash

```console
$ cd ide
$ make
$ make program       # ST-LINK (ST-LINK_CLI.exe)
```

### IDEs

| IDE                     | Path                  |
|-------------------------|-----------------------|
| SEGGER Embedded Studio  | ide/SES/              |
| IAR EWARM               | ide/EWARM/            |
| Keil MDK-ARM (uVision)  | ide/MDK-ARM/          |

## Calibration and Non-Volatile State

| Feature | Mechanism |
|---------|-----------|
| BMP180 PROM | CRC32-verified, stored in FLASH page |
| System status | nvStatus in backup register (RTC domain) |
| Cycle counter | RTC backup register |
| LSI calibration | Recalculated every 4 hours |

BMP180 calibration data is read from the sensor on first boot, verified with CRC32, written to FLASH, and loaded on subsequent boots to avoid repeated PROM reads.

## Project Structure

```
src/
+-- main.c              Application entry point and main loop
+-- adc.c               ADC driver (VREFINT, TSEN, VBAT, VLOAD)
+-- bmp180.c            BMP180 sensor driver + FLASH storage
+-- control.c           Power management, RTC, system control
inc/
+-- adc.h, bmp180.h, bh1750.h, si7021.h, nrf24l01.h
+-- i2c.h, spi.h, gpio.h, tim.h, flash.h, control.h
drv/
+-- inc/                CMSIS-CORE + STM32F031x6 headers
+-- src/                IAR startup file
ide/
+-- Makefile            GCC build system
+-- EWARM/              IAR Embedded Workbench project
+-- MDK-ARM/            Keil uVision project
+-- SES/                SEGGER Embedded Studio project
+-- Proteus/            Simulation files
+-- STM32F031F6Px_FLASH.ld  Linker script
+-- startup_stm32f031x6.s   GCC startup
```
