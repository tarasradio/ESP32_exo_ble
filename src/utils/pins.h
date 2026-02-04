#ifndef PINS_H
#define PINS_H

#include <Arduino.h>
#include "config.h"

void initPins() {
  pinMode(LED_OK_PIN, OUTPUT);
  pinMode(LED_REC_PIN, OUTPUT);
  pinMode(BTN_START_PIN, INPUT_PULLUP);
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
  pinMode(BATTERY_ADC_PIN, INPUT);
  digitalWrite(LED_OK_PIN, LOW);
  digitalWrite(LED_REC_PIN, LOW);
}

#endif