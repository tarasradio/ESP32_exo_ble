#ifndef SD_UTILS_H
#define SD_UTILS_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "../config.h"

bool initSDCard() {
  Serial.println("Initializing SD card...");
  SPI.begin(18, 19, 23, 5);
  delay(100);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD initialization failed!");
    return false;
  }
  uint8_t cardType = SD.cardType();
  Serial.printf("SD Card detected, type: %d\n", cardType);
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached or card not detected");
    return false;
  }
  Serial.println("SD card initialized successfully");
  return true;
}

#endif