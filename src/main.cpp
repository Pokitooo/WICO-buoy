#include <Arduino.h>
#include <lib_xcore>
#include <STM32FreeRTOS.h>

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

#include <SPI.h>
#include "RadioLib.h"

#include "DFRobot_PH.h"
#include "DFRobot_EC.h"
#include "DF_Sensor.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define UBLOX_CUSTOM_MAX_WAIT (250u)
#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

// I2C
constexpr PinName SDA1 = PB_7;
constexpr PinName SCL1 = PB_6;

// SPI
constexpr PinName MISO1 = PB_4;
constexpr PinName MOSI1 = PB_5;
constexpr PinName SCK1 = PA_5;

// LORA
constexpr PinName LORA_NSS = PB_12;
constexpr PinName LORA_NRST = PB_10;
constexpr PinName LORA_DIO = PB_9;
constexpr PinName LORA_BUSY = PB_8;

// DIGITAL
constexpr PinName DATA_TEMP = PB_0;
constexpr PinName DATA_RAIN = PA_6;

// ANALOG
constexpr PinName DATA_FLOW = PA_4;
constexpr PinName DATA_TURB = PA_3;
constexpr PinName DATA_DO = PA_2;
constexpr PinName DATA_EC = PA_1;
constexpr PinName DATA_PH = PA_0;

// i2c
TwoWire i2c1(pinNametoDigitalPin(SCL1), pinNametoDigitalPin(SDA1));
SFE_UBLOX_GNSS m10s;
Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_accel, *icm_gyro, *icm_mag;
SemaphoreHandle_t i2cMutex;

// spi
SPIClass spi1(pinNametoDigitalPin(MOSI1), pinNametoDigitalPin(MISO1), pinNametoDigitalPin(SCK1));

// LoRa
SPISettings lora_spi_settings(18'000'000, MSBFIRST, SPI_MODE0);

constexpr struct
{
  float center_freq = 921.000'000f; // MHz
  float bandwidth = 125.f;          // kHz
  uint8_t spreading_factor = 12;    // SF: 6 to 12
  uint8_t coding_rate = 8;          // CR: 5 to 8
  uint8_t sync_word = 0x12;         // Private SX1262
  int8_t power = 22;                // up to 22 dBm for SX1262
  uint16_t preamble_length = 16;
} params;

SX1262 lora = new Module(LORA_NSS, LORA_DIO, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

// Analog devices
constexpr size_t ADC_BITS(12);
constexpr float ADC_DIVIDER = ((1 << ADC_BITS - 1));
constexpr float VREF = 3300;
float voltagePH, voltageEC, voltageDO = 0;
DFRobot_EC ec;
DFRobot_PH ph;

// Digital devices
OneWire dataDS(DATA_TEMP);
DallasTemperature ds18(&dataDS);

// Data
struct Data
{
  uint32_t timestamp;
  uint8_t counter;

  float temp;

  float ecValue;
  float phValue;
  float doValue;

  double gps_latitude;
  double gps_longitude;
  float gps_altitude;

  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
} data;

// Variables

extern float readTemperature();

void setup()
{
  Serial.begin(460800);
  delay(2000);

  i2c1.begin();
  i2c1.setClock(300000u);
  i2cMutex = xSemaphoreCreateMutex();

  spi1.begin();

  // Onewire
  ds18.begin();

  // Analog
  analogReadResolution(ADC_BITS);
  ec.begin();
  ph.begin();

  // m10s (0x42)
  if (m10s.begin(i2c1))
  {
    m10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setNavigationFrequency(25, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setDynamicModel(DYN_MODEL_AUTOMOTIVE, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    Serial.println("GPS success");
  }

  // icm20949 (0x69)
  if (!icm.begin_I2C())
  {
    Serial.println("Failed to find ICM20948 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");
  icm_accel = icm.getAccelerometerSensor();
  icm_accel->printSensorDetails();

  icm_gyro = icm.getGyroSensor();
  icm_gyro->printSensorDetails();

  icm_mag = icm.getMagnetometerSensor();
  icm_mag->printSensorDetails();
}

void read_m10s(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
      if (m10s.getPVT())
      {
        data.timestamp = m10s.getUnixEpoch(UBLOX_CUSTOM_MAX_WAIT);
        data.gps_latitude = static_cast<double>(m10s.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_longitude = static_cast<double>(m10s.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_altitude = static_cast<float>(m10s.getAltitudeMSL(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
      }
      xSemaphoreGive(i2cMutex);
    }
    DELAY(500);
  }
}

void read_icm(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t mag;
      icm_accel->getEvent(&accel);
      icm_gyro->getEvent(&gyro);
      icm_mag->getEvent(&mag);

      data.acc_x = accel.acceleration.x;
      data.acc_y = accel.acceleration.y;
      data.acc_z = accel.acceleration.z;

      data.gyro_x = gyro.gyro.x;
      data.gyro_y = gyro.gyro.y;
      data.gyro_z = gyro.gyro.z;

      data.mag_x = mag.magnetic.x;
      data.mag_y = mag.magnetic.y;
      data.mag_z = mag.magnetic.z;

      xSemaphoreGive(i2cMutex);
    }
    DELAY(500);
  }
}

void read_probe(void *)
{
  for (;;)
  {
    data.temp = readTemperature();                                             // read your temperature sensor to execute temperature compensation
    voltagePH = analogRead(digitalPinToAnalogInput(pinNametoDigitalPin(DATA_PH))) / ADC_DIVIDER * VREF; // read the ph voltage
    data.phValue = ph.readPH(voltagePH, data.temp);                            // convert voltage to pH with temperature compensation
    voltageEC = analogRead(digitalPinToAnalogInput(pinNametoDigitalPin(DATA_EC))) / ADC_DIVIDER * VREF; // read the voltage
    data.ecValue = ec.readEC(voltageEC, data.temp);                            // convert voltage to EC with temperature compensation
    DOtemp = readTemperature();
    voltageDO = analogRead(digitalPinToAnalogInput(pinNametoDigitalPin(DATA_DO))) / ADC_DIVIDER * VREF;
    data.doValue = readDO(voltageDO, DOtemp);
    DELAY(1000);
  }
}

void loop()
{
  if (Serial.available() > 0)
  {
    char cmd = Serial.read();
    if (cmd == 'c')
    {
      ec.calibration(voltageEC, data.temp);
      ph.calibration(voltagePH, data.temp);
    }
  }
}

float readTemperature()
{
  ds18.requestTemperatures();
  data.temp = ds18.getTempCByIndex(0);
  return data.temp;
}
