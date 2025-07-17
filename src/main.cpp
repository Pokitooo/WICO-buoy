#include <Arduino.h>
#include "Arduino_Extended.h"
#include <lib_xcore>
#include <STM32FreeRTOS.h>
#include <EEPROM.h>

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

#include <SPI.h>
#include "RadioLib.h"
#include "lorawan_config.h"

#include "DFRobot_PH.h"
#include "DFRobot_EC.h"
#include "DF_Sensor.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "WQI.h"

#define UBLOX_CUSTOM_MAX_WAIT (250u)
#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))
#define ECADDR 0x0A

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
constexpr PinName DATA_FLOW = PA_4;

// ANALOG
constexpr PinName DATA_RAIN = PA_6;
constexpr PinName DATA_TURB = PA_3;
constexpr PinName DATA_DO = PA_2;
constexpr PinName DATA_EC = PA_1;
constexpr PinName DATA_PH = PA_0;

// GPIO
const int LED = PA7;
// i2c
TwoWire i2c1(pinNametoDigitalPin(SDA1), pinNametoDigitalPin(SCL1));
SFE_UBLOX_GNSS m10s;
Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_accel, *icm_gyro, *icm_mag;
SemaphoreHandle_t i2cMutex;

// spi
SPIClass spi1(pinNametoDigitalPin(MOSI1), pinNametoDigitalPin(MISO1), pinNametoDigitalPin(SCK1));

// LoRa
SPISettings lora_spi_settings(2'000'000, MSBFIRST, SPI_MODE0);

// regional choices: EU868, US915, AU915, AS923, AS923_2, AS923_3, AS923_4, IN865, KR920, CN470
const LoRaWANBand_t Region = US915;

// subband choice: for US915/AU915 set to 2, for CN470 set to 1, otherwise leave on 0
constexpr uint8_t subBand = 2;

uint64_t joinEUI = RADIOLIB_LORAWAN_JOIN_EUI;
uint64_t devEUI = RADIOLIB_LORAWAN_DEV_EUI;
uint8_t appKey[] = {RADIOLIB_LORAWAN_APP_KEY};
uint8_t nwkKey[] = {RADIOLIB_LORAWAN_NWK_KEY};

constexpr struct
{
  float center_freq = 915.000'000f; // MHz
  float bandwidth = 125.f;          // kHz
  uint8_t spreading_factor = 12;    // SF: 6 to 12
  uint8_t coding_rate = 8;          // CR: 5 to 8
  uint8_t sync_word = 0x12;         // Private SX1262
  int8_t power = 22;                // up to 22 dBm for SX1262
  uint16_t preamble_length = 16;
} params;

SX1262 lora = new Module(pinNametoDigitalPin(LORA_NSS), pinNametoDigitalPin(LORA_DIO), pinNametoDigitalPin(LORA_NRST), pinNametoDigitalPin(LORA_BUSY), spi1, lora_spi_settings);
LoRaWANNode node(&lora, &Region, subBand);

volatile bool tx_flag = false;
volatile int16_t lora_state = 0;
volatile int16_t node_state = 0;

// Analog devices
constexpr size_t ADC_BITS(12);
constexpr float ADC_DIVIDER = ((1 << ADC_BITS) - 1);
constexpr float VREF = 3300;
float voltagePH, voltageEC, voltageDO, voltageTURB, voltageRAIN = 0;
DFRobot_EC ec;
DFRobot_PH ph;

// Flow
uint16_t NbTopsFan; // measuring the rising edges of the signal

// Digital devices
OneWire DS18(pinNametoDigitalPin(DATA_TEMP));
DallasTemperature ds18(&DS18);

// Data
struct Data
{
  uint32_t timestamp;
  uint8_t counter;

  double gps_latitude;
  double gps_longitude;
  float gps_altitude;

  float temp;
  float ecValue;
  float phValue;
  float doValue;
  float turbValue;
  float rainValue;
  float flowValue;
  float WQI;
  String index;

  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
  uint16_t heading;
} data;

// Variable
static bool state;

uint16_t ls;

String constructed_data;

// Functions
extern void read_m10s(void *);

extern void read_icm(void *);

extern void read_probe(void *);

extern void read_flow(void *);

extern void calculate_wqi(void *);

extern void construct_data(void *);

extern void transmit_data(void *);

extern void printData(void *);

extern float readTemperature();

extern float readTurbidity(float voltageRatio);

extern void rpm();

extern void deleted();

void setup()
{
  Serial.begin(115200);
  delay(2000);

  // LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);
  delay(100);
  digitalWrite(LED, 0);

  // // i2c
  i2c1.begin();
  i2c1.setClock(300000u);
  i2cMutex = xSemaphoreCreateMutex();

  // SPI
  spi1.begin();

  // LoRa
  ls = lora.begin(params.center_freq,
                  params.bandwidth,
                  params.spreading_factor,
                  params.coding_rate,
                  params.sync_word,
                  params.power,
                  params.preamble_length);
  state = state || lora.explicitHeader();
  state = state || lora.setCRC(true);
  state = state || lora.setDio2AsRfSwitch();
  state = state || lora.autoLDRO();

  Serial.printf("SX1262 LoRa %s\n", lora_state == RADIOLIB_ERR_NONE ? "SUCCESS" : "FAILED");
  if (lora_state != RADIOLIB_ERR_NONE)
  {
    Serial.printf("Initialization failed! Error: %d\n", lora_state);
    digitalWrite(LED, 1);
    delay(1000);
    digitalWrite(LED, 0);
  }

  // node_state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  // node_state = node.activateOTAA();

  // Serial.printf("LoRaWAN Node %s\n", node_state == RADIOLIB_ERR_NONE ? "SUCCESS" : "FAILED");
  // if (node_state != RADIOLIB_ERR_NONE)
  // {
  //   Serial.printf("Initialization failed! Error: %d\n", node_state);
  //   digitalWrite(LED, 1);
  //   delay(1000);
  //   digitalWrite(LED, 0);
  // }

  // Onewire
  ds18.begin();

  // Digital
  pinMode(digitalPinToInterrupt(pinNametoDigitalPin(DATA_FLOW)), INPUT);
  attachInterrupt(digitalPinToInterrupt(pinNametoDigitalPin(DATA_FLOW)), rpm, RISING);

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
  else
  {
    Serial.print("GPS Failed");
  }

  // icm20949 (0x69)
  // if (icm.begin_I2C(0x69, &i2c1))
  // {
  //   icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  //   icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  //   icm_accel = icm.getAccelerometerSensor();
  //   icm_accel->printSensorDetails();
  //   icm_gyro = icm.getGyroSensor();
  //   icm_gyro->printSensorDetails();
  //   icm_mag = icm.getMagnetometerSensor();
  //   icm_mag->printSensorDetails();
  // }

  // Scheduler
  // xTaskCreate(read_m10s, "", 2048, nullptr, 2, nullptr);
  // xTaskCreate(read_icm, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(read_probe, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(read_flow, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(calculate_wqi, "", 1024, nullptr, 2, nullptr);
  xTaskCreate(construct_data, "", 1024, nullptr, 2, nullptr);
  xTaskCreate(transmit_data, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(printData, "", 1024, nullptr, 2, nullptr);
  vTaskStartScheduler();
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
        data.gps_altitude = static_cast<float>(m10s.getAltitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
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
    // TEMPERATURE
    readTemperature(); // read your temperature sensor to execute temperature compensation

    // PH
    voltagePH = analogRead(digitalPinToAnalogInput(pinNametoDigitalPin(DATA_PH))) / ADC_DIVIDER * VREF; // read the ph voltage
    data.phValue = ph.readPH(voltagePH, data.temp);                                                     // convert voltage to pH with temperature compensation

    // EC
    voltageEC = analogRead(digitalPinToAnalogInput(pinNametoDigitalPin(DATA_EC))) / ADC_DIVIDER * VREF; // read the voltage
    data.ecValue = ec.readEC(voltageEC, data.temp);                                                     // convert voltage to EC with temperature compensation

    // DO
    voltageDO = analogRead(digitalPinToAnalogInput(pinNametoDigitalPin(DATA_DO))) / ADC_DIVIDER * VREF;
    data.doValue = readDO(voltageDO, data.temp) + 1.25;

    // TURB
    voltageTURB = analogRead(digitalPinToAnalogInput(pinNametoDigitalPin(DATA_TURB))) / ADC_DIVIDER * VREF;
    data.turbValue = readTurbidity(voltageTURB);

    // RAIN
    // voltageRAIN = analogRead(digitalPinToAnalogInput(pinNametoDigitalPin(DATA_RAIN))) / ADC_DIVIDER * VREF;

    DELAY(1000);
  }
}

void read_flow(void *)
{
  for (;;)
  {
    NbTopsFan = 0;                                      // Set NbTops to 0 ready for flowValueulations
    interrupts();                                       // Enables interrupts
    DELAY(1000);                                        // Wait 1 second
    noInterrupts();                                     // Disable interrupts
    data.flowValue = (NbTopsFan * 60 / 7.5) * 0.002194; //(Pulse frequency x 60) / 7.5Q, = flow rate in L/hour
    DELAY(1000);
  }
}

void transmit_data(void *)
{
  for (;;)
  {
    node_state = node.sendReceive(constructed_data.c_str(), 1);
    // lora.startTransmit("HELLO");
    ++data.counter;
    Serial.println("sent");
    digitalToggle(LED);
    DELAY(uplinkIntervalSeconds * 1'000);
  }
}

void calculate_wqi(void *)
{
  for (;;)
  {
    data.WQI = calculateWQI(data.turbValue, data.doValue / 1000, data.ecValue, data.phValue, data.temp);
    data.index = evaluateWQI(data.WQI);
    DELAY(1000);
  }
}

void calculate_heading(void *)
{
  for (;;)
  {
    data.heading = atan2(data.mag_y, data.mag_x) * (180.0 / PI);
    if (data.heading < 0)
      data.heading += 360;
    DELAY(1000);
  }
}

void construct_data(void *)
{
  for (;;)
  {
    constructed_data = "";
    csv_stream_crlf(constructed_data)
        << "<10>"
        << data.counter
        << data.timestamp
        << String(data.gps_latitude, 6)
        << String(data.gps_longitude, 6)
        << String(data.gps_altitude, 4)
        << data.temp
        << data.ecValue
        << data.phValue
        << data.doValue
        << data.turbValue
        << data.rainValue
        << data.flowValue
        << data.WQI
        << data.index
        << data.acc_x
        << data.acc_y
        << data.acc_z
        << data.gyro_x
        << data.gyro_y
        << data.gyro_z
        << data.heading;
    DELAY(1000);
  }
}

void printData(void *)
{
  for (;;)
  {
    Serial.println("=== Sensor Data ===");

    Serial.print("Timestamp: ");
    Serial.println(data.timestamp);
    Serial.print("Counter: ");
    Serial.println(data.counter);

    Serial.print("GPS Latitude: ");
    Serial.println(data.gps_latitude, 8); // Keep high precision
    Serial.print("GPS Longitude: ");
    Serial.println(data.gps_longitude, 8);
    Serial.print("GPS Altitude: ");
    Serial.println(data.gps_altitude);

    Serial.print("Temperature: ");
    Serial.println(data.temp);
    Serial.print("EC Value: ");
    Serial.println(data.ecValue);
    Serial.print("pH Value: ");
    Serial.println(data.phValue);
    Serial.print("DO Value: ");
    Serial.println(data.doValue);
    Serial.print("Turbidity: ");
    Serial.println(data.turbValue);
    // Serial.print("Rain: ");
    // Serial.println(data.rainValue);
    Serial.print("Flow: ");
    Serial.println(data.flowValue);

    // Serial.print("Accelerometer X: ");
    // Serial.println(data.acc_x);
    // Serial.print("Accelerometer Y: ");
    // Serial.println(data.acc_y);
    // Serial.print("Accelerometer Z: ");
    // Serial.println(data.acc_z);

    // Serial.print("Gyroscope X: ");
    // Serial.println(data.gyro_x);
    // Serial.print("Gyroscope Y: ");
    // Serial.println(data.gyro_y);
    // Serial.print("Gyroscope Z: ");
    // Serial.println(data.gyro_z);

    // Serial.print("Magnetometer X: ");
    // Serial.println(data.mag_x);
    // Serial.print("Magnetometer Y: ");
    // Serial.println(data.mag_y);
    // Serial.print("Magnetometer Z: ");
    // Serial.println(data.mag_z);

    // Serial.print("Voltage pH: ");
    // Serial.println(voltagePH, 3); // Keep 3 decimals
    // Serial.print("Voltage EC: ");
    // Serial.println(voltageEC, 3);
    // Serial.print("Voltage DO: ");
    // Serial.println(voltageDO, 3);
    // Serial.print("Voltage TURB: ");
    // Serial.println(voltageTURB, 3);
    // Serial.print("Voltage RAIN: ");
    // Serial.println(voltageRAIN, 3);

    // Serial.print("Water Quality Index (WQI): ");
    // Serial.println(data.WQI);
    // Serial.print("INDEX: ");
    // Serial.println(data.index);

    // Serial.println("===================");
    DELAY(1000);
  }
}

void loop()
{
  ec.calibration(voltageEC, data.temp);
  ph.calibration(voltagePH, data.temp);
}

float readTemperature()
{
  ds18.requestTemperatures();
  data.temp = ds18.getTempCByIndex(DATA_TEMP);
  return data.temp;
}

float readTurbidity(float voltageRatio)
{
  for (int i = 0; i < numPoints - 1; ++i)
  {
    CalibrationPoint p1 = calibrationPoints[i];
    CalibrationPoint p2 = calibrationPoints[i + 1];

    if (voltageRatio >= p2.voltageRatio && voltageRatio <= p1.voltageRatio)
    {
      ntu = p1.ntu + (voltageRatio - p1.voltageRatio) * (p2.ntu - p1.ntu) / (p2.voltageRatio - p1.voltageRatio);
      return ntu;
    }
  }

  if (voltageRatio > calibrationPoints[0].voltageRatio)
    return calibrationPoints[0].ntu;
  if (voltageRatio < calibrationPoints[numPoints - 1].voltageRatio)
    return calibrationPoints[numPoints - 1].ntu;

  return 0;
}

void rpm() // This is the function that the interupt calls
{
  ++NbTopsFan; // This function measures the rising and falling edge of the hall effect sensors signal
}

void deleted()
{

  for (int i = 0; i < 8; i++)
  {
    EEPROM.write(ECADDR + i, 0xFF); // write defaullt value to the EEPROM
    delay(10);
  }

  int a = 0;
  while (a < 8)
  {
    static int a = 0, value = 0;
    value = EEPROM.read(ECADDR + a);
    Serial.print(a, ' ');
    Serial.print(ECADDR + a, HEX);
    Serial.print(":");
    Serial.print(value); // print the new value of EEPROM block used by EC meter. The correct is 255.
    Serial.println();
    delay(10);
    a = a + 1;
    if (a > 7)
      break;
  }
}