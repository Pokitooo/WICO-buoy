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

// how often to send an uplink - consider legal & FUP constraints - see notes
const uint32_t uplinkIntervalSeconds = 0UL * 60UL + 2UL; // minutes x seconds

#define RADIOLIB_LORAWAN_JOIN_EUI  0x8314738482174831

// the Device EUI & two keys can be generated on the TTN console
#ifndef RADIOLIB_LORAWAN_DEV_EUI   // Replace with your Device EUI
#define RADIOLIB_LORAWAN_DEV_EUI   0x70B3D57ED0071F79
#endif
#ifndef RADIOLIB_LORAWAN_APP_KEY   // Replace with your App Key
#define RADIOLIB_LORAWAN_APP_KEY   0xDB, 0x13, 0x6C, 0x08, 0xB3, 0xE8, 0x05, 0x96, 0x2F, 0xFA, 0x54, 0xBC, 0x11, 0x44, 0xA5, 0xC6

#endif
#ifndef RADIOLIB_LORAWAN_NWK_KEY   // Put your Nwk Key here
#define RADIOLIB_LORAWAN_NWK_KEY   0xDB, 0xD1, 0x6A, 0x54, 0x8E, 0xF0, 0x71, 0x37, 0x34, 0xD3, 0xF2, 0xD7, 0xBE, 0x10, 0xD0, 0x9F

#endif

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
