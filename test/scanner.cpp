#include <Arduino_Extended.h>

TwoWire i2c1(PB7, PB6);

void setup() {
    Serial.begin();
    i2c1.begin();
    pinMode(PA7, OUTPUT);
  }

void loop() {
    Serial.println("Hello");
    digitalToggle(PA7);

    i2c_detect(Serial, i2c1, 0x00, 127);
    delay(1000);
}