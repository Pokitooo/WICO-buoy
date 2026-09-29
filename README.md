# BUOY — Water Quality Monitoring Buoy

## Introduction

BUOY is firmware for a floating water-quality monitoring station built around an STM32F411CE microcontroller. It reads a set of water-quality probes, works out a Water Quality Index (WQI) on the device, and sends the results over LoRa/LoRaWAN (SX1262 radio, US915 band). The same readings are also printed to the USB serial console.

The firmware is written with the Arduino framework on PlatformIO and uses FreeRTOS to run each job (sensor reading, WQI calculation, packet building, transmission and logging) as its own task.

## Overview

### Hardware

| Subsystem | Part | Interface | Pins |
|---|---|---|---|
| MCU | STM32F411CEU6 @ 96 MHz | — | — |
| Radio | Semtech SX1262 (LoRa / LoRaWAN) | SPI1 | NSS `PB12`, NRST `PB10`, DIO1 `PB9`, BUSY `PB8`, SCK `PA5`, MISO `PB4`, MOSI `PB5` |
| GNSS | u-blox M10S (`0x42`) | I2C1 @ 300 kHz | SDA `PB7`, SCL `PB6` |
| IMU | TDK ICM-20948 (`0x69`) | I2C1 | shared with GNSS |
| Water temperature | DS18B20 | OneWire | `PB0` |
| pH | DFRobot pH probe | ADC | `PA0` |
| Conductivity (EC) | DFRobot EC probe | ADC | `PA1` |
| Dissolved oxygen (DO) | DFRobot DO probe | ADC | `PA2` |
| Turbidity | Analog turbidity sensor | ADC | `PA3` |
| Water flow | Hall-effect flow sensor | Interrupt (rising edge) | `PA4` |
| Rain | Analog rain sensor | ADC | `PA6` |
| Status LED | — | GPIO | `PA7` |

The ADC runs at 12-bit resolution with a 3300 mV reference.

### Firmware structure

`setup()` in [src/main.cpp](src/main.cpp) starts the buses and peripherals (I2C, SPI, SX1262, OneWire, the flow interrupt, the ADC and the pH/EC libraries, and the GNSS), creates the FreeRTOS tasks and then starts the scheduler.

| Task | Period | Purpose |
|---|---|---|
| `read_probe` | 1 s | Reads temperature, then pH, EC, DO and turbidity. The temperature reading is used to compensate pH, EC and DO. |
| `read_flow` | 2 s | Counts flow-sensor pulses over a 1 s window and converts the count to a flow rate (L/h). |
| `calculate_wqi` | 1 s | Computes the WQI from turbidity, DO, EC, pH and temperature ([include/WQI.h](include/WQI.h)) and assigns a quality class. |
| `construct_data` | 1 s | Builds a CSV payload (prefixed with `<10>`) from all fields in the `Data` struct. |
| `transmit_data` | `uplinkIntervalSeconds` | Sends the payload with `node.sendReceive()`, increments the packet counter and toggles the LED. |
| `printData` | 1 s | Prints a readable summary of the sensor values to the serial console at 115200 baud. |
| `read_m10s` | 500 ms | Reads GNSS time, latitude, longitude and altitude. *(Currently disabled.)* |
| `read_icm` | 500 ms | Reads accelerometer, gyroscope and magnetometer data. *(Currently disabled.)* |

The GNSS and IMU tasks share the I2C bus, and a FreeRTOS mutex (`i2cMutex`) stops them from using it at the same time.

`loop()` runs the DFRobot pH/EC calibration routines, which accept calibration commands over the serial port.

### Water Quality Index

The WQI is a weighted index calculated from four parameters: turbidity, dissolved oxygen, conductivity and pH. The DO standard is adjusted for water temperature. The resulting value falls into one of these classes:

| WQI | Class |
|---|---|
| < 25 | Excellent water quality |
| 25 – 49 | Good water quality |
| 50 – 74 | Poor water quality |
| 75 – 99 | Very poor water quality |
| ≥ 100 | Highly contaminated water |

### Telemetry payload

Each uplink is a single CSV line with the fields in this order:

```
<10>, counter, timestamp, latitude, longitude, altitude,
temp, EC, pH, DO, turbidity, rain, flow, WQI, WQI class,
acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, heading
```

### Radio configuration

- **LoRa PHY:** 915 MHz, 125 kHz bandwidth, SF12, CR 4/8, sync word `0x12`, 22 dBm, 16-symbol preamble, explicit header, CRC enabled
- **LoRaWAN:** US915, sub-band 2. The keys and uplink interval are set in [include/lorawan_config.h](include/lorawan_config.h).

### Current status

- The LoRaWAN OTAA join (`beginOTAA` / `activateOTAA`) is commented out.
- The GNSS is initialised, but its reading task is not started, so the GPS fields stay at zero.
- The IMU initialisation and reading task are commented out, and `calculate_heading` is never scheduled.
- The rain sensor is wired but not read.