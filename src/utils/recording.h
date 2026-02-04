#ifndef RECORDING_H
#define RECORDING_H

#include <Arduino.h>
#include <SD.h>
#include "../config.h"
#include "../globals.h"

uint8_t calculateBatteryPercent(float voltage);
String formatTime(unsigned long milliseconds);
bool getTimestampForFilename(char* timestampStr, size_t maxLen);
void startRecording();
void stopRecording();

// Константы для батареи
#define BATTERY_MAX_VOLTAGE 25.2f  // Максимальное напряжение для 6s
#define BATTERY_MIN_VOLTAGE 21.0f  // Минимальное напряжение для 6s

// #define BATTERY_MAX_VOLTAGE 16.8f  // Максимальное напряжение для 4s
// #define BATTERY_MIN_VOLTAGE 14.0f  // Минимальное напряжение для 4s

// Функция расчета процента заряда батареи
uint8_t calculateBatteryPercent(float voltage) {
    if (voltage >= BATTERY_MAX_VOLTAGE) return 100;
    if (voltage <= BATTERY_MIN_VOLTAGE) return 0;
    
    float percent = ((voltage - BATTERY_MIN_VOLTAGE) / 
                    (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0f;
    return (uint8_t)constrain(percent, 0, 100);
}

// Функция форматирования времени
String formatTime(unsigned long milliseconds) {
    unsigned long seconds = milliseconds / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    
    seconds %= 60;
    minutes %= 60;
    
    char buffer[20];
    if (hours > 0) {
        snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu", hours, minutes, seconds);
    } else {
        snprintf(buffer, sizeof(buffer), "%02lu:%02lu", minutes, seconds);
    }
    return String(buffer);
}

// Функция получения времени - используем millis() вместо системного времени
bool getTimestampForFilename(char* timestampStr, size_t maxLen) {
    uint32_t time_ms = millis();
    uint32_t hours = (time_ms / 3600000) % 24;
    uint32_t minutes = (time_ms / 60000) % 60;
    uint32_t seconds = (time_ms / 1000) % 60;
    uint32_t milliseconds = time_ms % 1000;
    
    snprintf(timestampStr, maxLen, "%02lu%02lu%02lu_%03lu",
             hours, minutes, seconds, milliseconds);
    return true;
}


void startRecording() {
    Serial.println("=== START RECORDING ===");
    
    if (xSemaphoreTake(xRecordMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Serial.println("ERROR: Timeout acquiring record mutex!");
        return;
    }
    
    if (recording_is_run) {
        Serial.println("Recording already in progress");
        xSemaphoreGive(xRecordMutex);
        return;
    }
    
    if (xSemaphoreTake(xSDMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Serial.println("ERROR: Timeout acquiring SD mutex!");
        xSemaphoreGive(xRecordMutex);
        return;
    }
    
    try {
        char filename[64];
        char timestamp[32];
        
        if (getTimestampForFilename(timestamp, sizeof(timestamp))) {
            snprintf(filename, sizeof(filename), "/data_%s.csv", timestamp);
        } else {
            // Резервный вариант
            snprintf(filename, sizeof(filename), "/data_%lu.csv", millis());
        }
        
        // Проверяем, не существует ли уже файл с таким именем
        int counter = 1;
        char finalFilename[64];
        strcpy(finalFilename, filename);
        
        while (SD.exists(finalFilename)) {
            Serial.printf("File %s already exists, trying alternative name\n", finalFilename);
            
            if (getTimestampForFilename(timestamp, sizeof(timestamp))) {
                snprintf(finalFilename, sizeof(finalFilename), "/data_%s_%d.csv", timestamp, counter);
            } else {
                snprintf(finalFilename, sizeof(finalFilename), "/data_%lu_%d.csv", millis(), counter);
            }
            counter++;
            
            if (counter > 100) {
                Serial.println("Too many duplicate files, using millis only");
                snprintf(finalFilename, sizeof(finalFilename), "/data_%lu.csv", millis());
                break;
            }
        }
        
        Serial.printf("Creating file: %s\n", finalFilename);
        dataFile = SD.open(finalFilename, FILE_WRITE);
        
        if (!dataFile) {
            Serial.println("ERROR: Failed to open file!");
        } else {
            // Сохраняем имя файла
            if (xSemaphoreTake(xBLEMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                strncpy((char*)current_filename, finalFilename + 1, 63); // Пропускаем первый '/'
                current_filename[63] = '\0';
                
                // Устанавливаем время начала записи
                recording_start_time = millis();
                recording_duration = 0;
                xSemaphoreGive(xBLEMutex);
            }

            // Записываем заголовок CSV (раскомментируйте если нужно)
            // dataFile.println("Timestamp(ms),Left_Angle,Right_Angle");
            dataFile.flush();
            recording_is_run = true;
            Serial.println("Recording started successfully");

            // Отправляем BLE уведомление
            if (pRecordControlCharacteristic) {
                pRecordControlCharacteristic->setValue("Recording STARTED");
                pRecordControlCharacteristic->notify();
            }
            
            // Сразу выводим статус CAN
            // debugCANStatus();
        }
    } catch (...) {
        Serial.println("EXCEPTION during recording start");
        recording_is_run = false;
        if (dataFile) dataFile.close();
    }
    
    xSemaphoreGive(xSDMutex);
    xSemaphoreGive(xRecordMutex);
    Serial.println("=== START RECORDING COMPLETE ===");
}

void stopRecording() {
    Serial.println("=== STOP RECORDING ===");

    if (xSemaphoreTake(xRecordMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Serial.println("ERROR: Timeout acquiring record mutex!");
        return;
    }

    if (!recording_is_run) {
        Serial.println("Recording not running");
        xSemaphoreGive(xRecordMutex);
        return;
    }

    if (xSemaphoreTake(xSDMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Serial.println("ERROR: Timeout acquiring SD mutex!");
        xSemaphoreGive(xRecordMutex);
        return;
    }

    // Обновляем длительность записи
    if (xSemaphoreTake(xBLEMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        recording_duration = millis() - recording_start_time;
        xSemaphoreGive(xBLEMutex);
    }

    try {
        recording_is_run = false;
        if (dataFile) {
            dataFile.flush();
            dataFile.close();
            Serial.println("File closed successfully");
        }
        digitalWrite(LED_REC_PIN, LOW);
        Serial.println("Recording stopped successfully");
        
        // Отправляем BLE уведомление
        if (pRecordControlCharacteristic) {
            pRecordControlCharacteristic->setValue("Recording STOPPED");
            pRecordControlCharacteristic->notify();
        }
    } catch (...) {
        Serial.println("EXCEPTION during stop recording");
    }
    xSemaphoreGive(xSDMutex);
    xSemaphoreGive(xRecordMutex);
    Serial.println("=== STOP RECORDING COMPLETE ===");
}

#endif