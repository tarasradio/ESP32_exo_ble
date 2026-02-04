#ifndef DISPLAY_UTILS_H
#define DISPLAY_UTILS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "../config.h"
#include "../globals.h"

bool initDisplay() {
  Serial.println("Initializing display...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    return false;
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("ESP32 Hub FreeRTOS");
  display.display();
  Serial.println("Display initialized successfully");
  return true;
}

#endif