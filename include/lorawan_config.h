#ifndef _RADIOLIB_EX_LORAWAN_CONFIG_H
#define _RADIOLIB_EX_LORAWAN_CONFIG_H

#include <RadioLib.h>

// if you have RadioBoards (https://github.com/radiolib-org/RadioBoards)
// and are using one of the supported boards, you can do the following:
/*
#define RADIO_BOARD_AUTO
#include <RadioBoards.h>

Radio radio = new RadioModule();
*/

extern SX1262 lora;

// how often to send an uplink - consider legal & FUP constraints - see notes
const uint32_t uplinkIntervalSeconds = 0UL * 60UL + 1UL; // minutes x seconds

#ifndef RADIOLIB_LORAWAN_DEV_ADDR   // Replace with your DevAddr
#define RADIOLIB_LORAWAN_DEV_ADDR   0x260BC6BE
#endif

#ifndef RADIOLIB_LORAWAN_FNWKSINT_KEY   // Replace with your FNwkSInt Key
#define RADIOLIB_LORAWAN_FNWKSINT_KEY   0xAB, 0x98, 0x67, 0xED, 0xA8, 0xD4, 0xD4, 0x69, 0x10, 0xF1, 0x46, 0x38, 0x12, 0xCE, 0xBA, 0xA7

#endif
#ifndef RADIOLIB_LORAWAN_SNWKSINT_KEY   // Replace with your SNwkSInt Key
#define RADIOLIB_LORAWAN_SNWKSINT_KEY   0x77, 0x54, 0x49, 0xBC, 0x7A, 0x66, 0xB7, 0x8E, 0xA5, 0xBA, 0x0C, 0x1A, 0xB2, 0x21, 0xB2, 0xAC

#endif
#ifndef RADIOLIB_LORAWAN_NWKSENC_KEY   // Replace with your NwkSEnc Key
#define RADIOLIB_LORAWAN_NWKSENC_KEY   0xB9, 0xB1, 0x58, 0x9B, 0x2C, 0x0C, 0x1F, 0x42, 0xB4, 0xA4, 0x63, 0x77, 0x8F, 0x57, 0x2B, 0x41

#endif
#ifndef RADIOLIB_LORAWAN_APPS_KEY   // Replace with your AppS Key
#define RADIOLIB_LORAWAN_APPS_KEY   0x14, 0xE9, 0xE1, 0x9E, 0xE9, 0x13, 0x4A, 0x16, 0x40, 0x4E, 0xCC, 0xF5, 0x8F, 0xFE, 0x1A, 0x0A

#endif

// for the curious, the #ifndef blocks allow for automated testing &/or you can
// put your EUI & keys in to your platformio.ini - see wiki for more tips

// regional choices: EU868, US915, AU915, AS923, AS923_2, AS923_3, AS923_4, IN865, KR920, CN470
LoRaWANBand_t Region = AS923;

// subband choice: for US915/AU915 set to 2, for CN470 set to 1, otherwise leave on 0
constexpr uint8_t subBand = 0;

// ============================================================================
// Below is to support the sketch - only make changes if the notes say so ...

// copy over the EUI's & keys in to the something that will not compile if incorrectly formatted
inline uint32_t devAddr = RADIOLIB_LORAWAN_DEV_ADDR;
inline uint8_t fNwkSIntKey[] = {RADIOLIB_LORAWAN_FNWKSINT_KEY};
inline uint8_t sNwkSIntKey[] = {RADIOLIB_LORAWAN_SNWKSINT_KEY};
inline uint8_t nwkSEncKey[] = {RADIOLIB_LORAWAN_NWKSENC_KEY};
inline uint8_t appSKey[] = {RADIOLIB_LORAWAN_APPS_KEY};

// create the LoRaWAN node
inline LoRaWANNode node(&lora, &Region, subBand);

// result code to text - these are error codes that can be raised when using LoRaWAN
// however, RadioLib has many more - see https://jgromes.github.io/RadioLib/group__status__codes.html for a complete list
String stateDecode(const int16_t result) {
    switch (result) {
        case RADIOLIB_ERR_NONE:
            return "ERR_NONE";
        case RADIOLIB_ERR_CHIP_NOT_FOUND:
            return "ERR_CHIP_NOT_FOUND";
        case RADIOLIB_ERR_PACKET_TOO_LONG:
            return "ERR_PACKET_TOO_LONG";
        case RADIOLIB_ERR_RX_TIMEOUT:
            return "ERR_RX_TIMEOUT";
        case RADIOLIB_ERR_MIC_MISMATCH:
            return "ERR_MIC_MISMATCH";
        case RADIOLIB_ERR_INVALID_BANDWIDTH:
            return "ERR_INVALID_BANDWIDTH";
        case RADIOLIB_ERR_INVALID_SPREADING_FACTOR:
            return "ERR_INVALID_SPREADING_FACTOR";
        case RADIOLIB_ERR_INVALID_CODING_RATE:
            return "ERR_INVALID_CODING_RATE";
        case RADIOLIB_ERR_INVALID_FREQUENCY:
            return "ERR_INVALID_FREQUENCY";
        case RADIOLIB_ERR_INVALID_OUTPUT_POWER:
            return "ERR_INVALID_OUTPUT_POWER";
        case RADIOLIB_ERR_NETWORK_NOT_JOINED:
            return "RADIOLIB_ERR_NETWORK_NOT_JOINED";
        case RADIOLIB_ERR_DOWNLINK_MALFORMED:
            return "RADIOLIB_ERR_DOWNLINK_MALFORMED";
        case RADIOLIB_ERR_INVALID_REVISION:
            return "RADIOLIB_ERR_INVALID_REVISION";
        case RADIOLIB_ERR_INVALID_PORT:
            return "RADIOLIB_ERR_INVALID_PORT";
        case RADIOLIB_ERR_NO_RX_WINDOW:
            return "RADIOLIB_ERR_NO_RX_WINDOW";
        case RADIOLIB_ERR_INVALID_CID:
            return "RADIOLIB_ERR_INVALID_CID";
        case RADIOLIB_ERR_UPLINK_UNAVAILABLE:
            return "RADIOLIB_ERR_UPLINK_UNAVAILABLE";
        case RADIOLIB_ERR_COMMAND_QUEUE_FULL:
            return "RADIOLIB_ERR_COMMAND_QUEUE_FULL";
        case RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND:
            return "RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND";
        case RADIOLIB_ERR_JOIN_NONCE_INVALID:
            return "RADIOLIB_ERR_JOIN_NONCE_INVALID";
        case RADIOLIB_ERR_DWELL_TIME_EXCEEDED:
            return "RADIOLIB_ERR_DWELL_TIME_EXCEEDED";
        case RADIOLIB_ERR_CHECKSUM_MISMATCH:
            return "RADIOLIB_ERR_CHECKSUM_MISMATCH";
        case RADIOLIB_ERR_NO_JOIN_ACCEPT:
            return "RADIOLIB_ERR_NO_JOIN_ACCEPT";
        case RADIOLIB_LORAWAN_SESSION_RESTORED:
            return "RADIOLIB_LORAWAN_SESSION_RESTORED";
        case RADIOLIB_LORAWAN_NEW_SESSION:
            return "RADIOLIB_LORAWAN_NEW_SESSION";
        case RADIOLIB_ERR_NONCES_DISCARDED:
            return "RADIOLIB_ERR_NONCES_DISCARDED";
        case RADIOLIB_ERR_SESSION_DISCARDED:
            return "RADIOLIB_ERR_SESSION_DISCARDED";
    }
    return "See https://jgromes.github.io/RadioLib/group__status__codes.html";
}

// helper function to display any issues
void debug(bool failed, const __FlashStringHelper *message, int state, bool halt) {
    if (failed) {
        Serial.print(message);
        Serial.print(" - ");
        Serial.print(stateDecode(state));
        Serial.print(" (");
        Serial.print(state);
        Serial.println(")");
        while (halt) { delay(1); }
    }
}

// helper function to display a byte array
void arrayDump(uint8_t *buffer, uint16_t len) {
    for (uint16_t c = 0; c < len; c++) {
        char b = buffer[c];
        if (b < 0x10) { Serial.print('0'); }
        Serial.print(b, HEX);
    }
    Serial.println();
}

#endif
