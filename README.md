# STM32F030F4P6-WIRELESS-MULTISENSOR

[![Build](https://github.com/a5021/STM32F030F4P6-WIRELESS-MULTISENSOR/actions/workflows/build.yml/badge.svg)](https://github.com/a5021/STM32F030F4P6-WIRELESS-MULTISENSOR/actions/workflows/build.yml) [![MCU](https://img.shields.io/badge/MCU-STM32F030F4P6-00A9E0)]() [![Radio](https://img.shields.io/badge/Radio-nRF24L01-00A9E0)]() [![Sensors](https://img.shields.io/badge/Sensors-BMP180_SI7021_BH1750-00A9E0)]() [![IDE](https://img.shields.io/badge/IDE-Make%20%7C%20EWARM%20%7C%20MDK--ARM%20%7C%20SES-00A9E0)]()

Multi-sensor wireless node based on STM32F030F4P6. Reads environmental data from BMP180 (pressure, temperature), SI7021 (humidity, temperature), and BH1750 (illuminance), monitors battery and die temperature via ADC, and transmits a composite packet over an nRF24L01+ radio link. Designed for ultra-low-power battery operation with adaptive duty cycling. The datagrams are received by an ATmega8-based repeater ([ATMEGA8-NRF24L01-REPEATER](https://github.com/a5021/ATMEGA8-NRF24L01-REPEATER)) and can be decoded by an STM8S-based receiver ([STM8S-NRF24L01-RECEIVER](https://github.com/a5021/STM8S-NRF24L01-RECEIVER)).

## Features

- Register-level peripheral drivers with DMA for ADC and BMP180 PROM reads; I2C and SPI interrupt-driven, RTC via EXTI
- Three environmental sensors on shared I2C bus:
  - Bosch BMP180 - pressure and temperature (int32 compensation)
  - Silicon Labs SI7021 - humidity and temperature
  - Rohm BH1750 - illuminance (dynamic resolution selection)
- nRF24L01+ radio in Enhanced ShockBurst mode, variable payload (5-10 bytes)
- ADC VREFINT, VBAT, VLOAD monitoring with DMA and auto-off
- Adaptive wake interval: 1-3 minutes based on VBAT level
- Power cycle of sensor rail on any reset without prior cycle (load switch PF0 + DC-DC PA0)
- CRC32-verified BMP180 calibration data stored in FLASH page
- Standby mode between cycles (RTC alarm wake)
- LSI calibration routine for RTC timing accuracy
- Periodic maintenance events keyed to wake-cycle counters, approximating hourly / daily / weekly / monthly at the 1-minute interval (longer as the adaptive wake interval increases)
- Dual-speed clocking: 8 MHz HSI for sensors/SPI, 32 MHz PLL burst for payload preparation
- NRF24L01+ auto-reinit on two consecutive TX failures
- Exception handlers (HardFault, NMI, SVC, PendSV) force Standby to prevent runaway
- BH1750 adaptive resolution: disabled (0 lux), HiRes II (<15 lx), HiRes I (15-199 lx), LowRes (>200 lx)
- Aggressive power saving: clock gating, GPIO analog mode, WFE, dual-speed PLL,
  DMA, NRF24L01+ NoACK/variable payload, sensor rail disconnect, ADC auto-off,
  I2C/SPI disable between operations

## Hardware Specification

| Component | Detail |
|-----------|--------|
| MCU | STMicroelectronics STM32F030F4P6 (ARM Cortex-M0, 16 KB Flash, 4 KB RAM) |
| Radio | Nordic nRF24L01+ (SPI1, 2 Mbps, channel 99, lowest TX power) |
| Pressure | Bosch BMP180 (I2C addr 0x77, OSS=2 / high resolution, 13.5 ms) |
| Humidity | Silicon Labs SI7021 (I2C addr 0x40) |
| Light | Rohm BH1750FVI (I2C addr 0x23, 0.5-65535 lx) |
| Power | DC-DC converter (PA0), load switch (PF0), VDD monitoring via PVD |
| Debug | SWD on PA13/PA14 |

## Pin Assignment

| Signal | Pin  | Peripheral    | Notes                         |
|--------|------|---------------|-------------------------------|
| I2C_SCL| PA9  | I2C1          | Shared bus, 400 kHz           |
| I2C_SDA| PA10 | I2C1          |                               |
| SPI_SCK| PA5  | SPI1          | nRF24L01+, 4 MHz (PCLK/2)     |
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

Variable-length payload transmitted via nRF24L01+ in Enhanced ShockBurst mode
(NoACK, dynamic payload, 3-byte address, channel 99). Total size depends on
data available: 5 B, 7 B, 8 B, or 10 B.

The TX/RX address is left at the nRF24L01+ power-on default
(`0xE7E7E7`, 3-byte width) and is not explicitly reprogrammed by the
firmware; the receiver/repeater must use the same default address.

### Base packet (5 B, always present)

| Offset | Bits      | Field           | Description                                   |
|--------|-----------|-----------------|-----------------------------------------------|
| 0      | [0]       | TX Status       | Last TX attempt (0=fail, 1=success)           |
| 0      | [4:1]     | Packet ID       | 4-bit field; lower 3 bits = rolling ID from cycle counter bits [2:0], bit [4] reserved (always 0) |
| 0      | [5]       | Sensor Status   | I2C sensors I/O (0=error, 1=success)          |
| 0      | [7:6]     | BMP180 Temp[9:8]| MSB fragment                                  |
| 1      | [7:0]     | BMP180 Temp[7:0]| LSBs, signed, -512..511 scale                 |
| 2      | [6:0]     | BMP180 Pressure | mmHg - 700, unsigned, 0-127                   |
| 2      | [7]       | SI7021 Humi[6]  | MSB bit of humidity                           |
| 3      | [5:0]     | SI7021 Humi[5:0]| LSBs, max 127 %RH                             |
| 3      | [7:6]     | SI7021 Temp[9:8]| MSB fragment                                  |
| 4      | [7:0]     | SI7021 Temp[7:0]| LSBs, signed, C x 10                          |

### ADC extension (+3 B, when PKT_ID == 0, total 8 B)

| Offset | Bits      | Field           | Description                                   |
|--------|-----------|-----------------|-----------------------------------------------|
| 5      | [6:0]     | ADC Temp        | Die temperature, signed C                     |
| 5      | [7]       | VBAT[8]         | MSB of battery (9 bits total)                 |
| 6      | [7:0]     | VBAT[7:0]       | LSBs, mV                                      |
| 7      | [7:0]     | VCC             | VREFINT-calibrated VDD - 160                  |

### BH1750-only extension (+2 B at offset 5-6, replaces ADC, total 7 B)
#### (PKT_ID != 0 and BH1750 not disabled)

| Offset | Bits      | Field           | Description                                   |
|--------|-----------|-----------------|-----------------------------------------------|
| 5      | [7:0]     | BH1750[7:0]     | Illuminance LSB, little-endian                |
| 6      | [7:0]     | BH1750[15:8]    | Illuminance MSB, lux x 10/12                  |

### Full extension (+5 B, total 10 B, when PKT_ID == 0 and BH1750 not disabled)
ADC occupies bytes 5-7, BH1750 occupies bytes 8-9:

| Offset | Size      | Field           | Description                                   |
|--------|-----------|-----------------|-----------------------------------------------|
| 5-7    | 3 B       | (same as ADC)   | ADC Temp + VBAT + VCC                         |
| 8-9    | 2 B       | BH1750          | Illuminance, lux x 10/12, little-endian       |

Total payload size: **5 B** (base only), **7 B** (base + BH1750),
**8 B** (base + ADC), **10 B** (base + ADC + BH1750).
BH1750 is measured on every cycle unless set to DISABLED, and forced every
8th packet regardless of disable state. HiRes Mode 2 requires ~120 ms
conversion time; the MCU sleeps during this interval.

NRF24L01+ is woken via EXTI1 falling edge on PF1 (`__WFE()`), which
resumes execution immediately when the radio asserts IRQ after TX.

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
| Standby (RTC alarm) | ~2.5 uA     | Cortex-M0 Standby + RTC on LSI    |
| Sensor rail off     | ~1.5 uA     | Load switch PF0 disables sensors  |

Wake interval is adaptive based on battery voltage:

| VBAT (divider) | Interval |
|----------------|----------|
| >2.05 V        | 1 min    |
| 1.45-2.05 V    | 1.5 min  |
| 1.15-1.45 V    | 2 min    |
| 1.05-1.15 V    | 2.5 min  |
| <1.05 V        | 3 min    |

When `adc_vbat == adc_vcc`, the DC-DC converter is off and the MCU is
powered directly from the battery through the load switch. If VBAT drops
below 2.01 V (divider reading), PA0 pulls the DC-DC enable line LOW,
activating the boost converter which supplies 3.3 V to the MCU and sensor
rail. When DC-DC is active (`adc_vbat != adc_vcc`) and VBAT rises above
2.26 V, the converter is disabled to save power, and the system returns to
direct battery operation.

During power cycle the load switch (PF0) is turned off, the rail is
discharged until `adc_read_vload()` drops below 50 mV, then DC-DC is
enabled and the rail is re-enabled once it rises above 240 mV.

### Power Optimization Techniques

The firmware applies numerous low-power tricks throughout:

| Technique | Where | Purpose |
|-----------|-------|---------|
| **Clock gating** | Every function toggles `RCC->AHBENR` / `RCC->APB2ENR` | Only peripherals in use receive a clock |
| **GPIO analog mode** | All pins set to `ANALOG_MODE_FOR_ALL_PINS` (0xFFFFFFFF) when idle | Eliminates floating-pin leakage currents |
| **WFE instead of WFI** | `__WFE()` for sleep, delays, NRF IRQ wait | WFE is edge-triggered, doesn't wake on pending interrupts |
| **Dual-speed MCU** | 8 MHz HSI for sensors/SPI, 32 MHz PLL only for payload packing | Faster payload prep reduces active time |
| **Lowest speed for delays** | `RUN_MCU_AT(LOWEST_FREQ)` = HSI/512 (15.6 kHz) | Timer delays at minimal power |
| **DMA transfers** | ADC uses DMA ch1, BMP180 PROM uses DMA ch3 | CPU sleeps (`__WFE()`) during DMA, wakes on transfer complete |
| **I2C disable between ops** | `i2c_disable()` between sensor reads, `i2c_enable()` only when needed | I2C peripheral + pins powered down |
| **SPI disable** | `RCC->APB2ENR = 0` when SPI not active | SPI peripheral clock removed |
| **ADC auto-off** | `ADC_CFGR1_AUTOOFF` bit | ADC self-disables between conversions |
| **HSI14 on-demand** | `RCC_CR2_HSI14ON` enabled only during ADC, disabled after | 14 MHz RC oscillator off when ADC idle |
| **NRF24L01+ power down** | `NRF24_POWER_DOWN()` after each TX cycle | Radio oscillator off, ~900 nA standby |
| **NRF24L01+ NoACK** | `W_TX_PAYLOAD_NOACK` command, `EN_PAYLOAD_NOACK` | No ACK wait reduces TX time |
| **Variable payload size** | 5-10 bytes depending on available data | Shorter packets = shorter TX time |
| **Sensor rail disconnect** | Load switch PF0 off between cycles | Entire sensor I2C bus powered off |
| **BH1750 disable at 0 lux** | `NV_BH1750_DISABLED` when luminosity = 0 | Light sensor not re-measured |
| **ADC skip** | `NV_SKIP_ADC` flag, re-measure only every 8th cycle when stable | VBAT/VREFINT measured infrequently |
| **PVD for VDD detection** | `check_vdd()` uses Programmable Voltage Detector | Ensures VDD stable before proceeding |
| **Standby between cycles** | Cortex-M0 Standby + RTC on LSI | Lowest possible MCU power mode (~2.5 uA) |
| **RTC bypass shadow** | `RTC_CR_BYPSHAD` | Direct register read, no synchronization delay |

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
| LSI calibration | Recalculated every 256 wake cycles (~4-13 h at the 1-3 min adaptive interval) via TIM14 input-capture of LSI using the internal TIM14_OR TI1_RMP remap (LSI routed internally to TIM14_CH1; no MCO/PA8 pin involved), 16 samples with skip-8 warm-up |

BMP180 calibration data is read from the sensor on first boot, verified with CRC32, written to FLASH, and loaded on subsequent boots to avoid repeated PROM reads.

ADC buffers use a skip-first-2 approach for VREFINT: `vref_buf[18]` stores
VREFINT in the first 2 (skip) + 16 elements; VBAT and temperature sensor
use separate 16-element buffers. Averages are computed over 16 samples.

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


## Related Projects

- [ATMEGA8-NRF24L01-REPEATER](https://github.com/a5021/ATMEGA8-NRF24L01-REPEATER) — wireless repeater that receives and retransmits this sensor's datagrams
- [STM8S-NRF24L01-RECEIVER](https://github.com/a5021/STM8S-NRF24L01-RECEIVER) — STM8S-based receiver that decodes the relayed data
