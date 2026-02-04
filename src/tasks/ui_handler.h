#ifndef UI_HANDLER_H
#define UI_HANDLER_H

#include <Arduino.h>
#include "../config.h"
#include "../globals.h"
#include "../utils/recording.h"

void taskUIHandler(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static unsigned long lastStartPress = 0;
  static unsigned long lastStopPress = 0;
  for (;;) {
    xTaskDelayUntil(&xLastWakeTime, xFrequency);
    buttonStart.tick();
    buttonStop.tick();
    if (buttonStart.isClick() && (millis() - lastStartPress > 1000)) {
      Serial.println("Start button clicked");
      startRecording();
      lastStartPress = millis();
    }
    if (buttonStop.isClick() && (millis() - lastStopPress > 1000)) {
      Serial.println("Stop button clicked");
      stopRecording();
      lastStopPress = millis();
    }
  }
}

#endif