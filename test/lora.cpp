#include <Arduino_Extended.h>
#include <RadioLib.h>

#define USE_FREERTOS 1
#if defined(USE_FREERTOS) && USE_FREERTOS
#define DELAY_MS(MS) vTaskDelay(pdMS_TO_TICKS(MS))
#else
#define DELAY_MS(MS) delay(MS)
#endif

// SPI
SPIClass spi1(PB5, PB4, PA5);

// LoRa data
volatile bool tx_flag = false;
String rx_buf;

// LoRa
SPISettings lora_spi_settings(18'000'000, MSBFIRST, SPI_MODE0);

// LoRa Parameters
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

// SX1262 pin connections (adjust pins for your board)
#define LORA_DIO1 PB9
#define LORA_NSS PB12
#define LORA_BUSY PB8
#define LORA_NRST PB10

// Initialize SX1262 module
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

void setup()
{
    static bool state;
    pinMode(PB5, OUTPUT);
    Serial.begin();
    spi1.begin();

    delay(2000);
    // Serial.println("Hi!");

    // state = sd.begin(sd_config);
    // Serial.printf("SD CARD: %s\n", state ? "SUCCESS" : "FAILED");

    int16_t ls = lora.begin(params.center_freq,
                            params.bandwidth,
                            params.spreading_factor,
                            params.coding_rate,
                            params.sync_word,
                            params.power,
                            params.preamble_length);
    state = state || lora.explicitHeader();
    state = state || lora.setCRC(true);
    state = state || lora.autoLDRO();

    if (ls == RADIOLIB_ERR_NONE)
    {
        Serial.println("SX1262 initialized successfully!");
    }
    else
    {
        Serial.print("Initialization failed! Error: ");
        Serial.println(ls);
        while (true)
            ;
    }

    rx_buf.reserve(300);
}

void loop()
{
    lora.startTransmit("HELLO");
    delay(1000);
}