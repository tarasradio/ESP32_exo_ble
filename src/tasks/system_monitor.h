#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "../config.h"
#include "../globals.h"
#include "odrive/utils/odrive_can.h"

// Задача системного мониторинга
void taskSystemMonitor(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(500);   // Увеличили частоту обновления (1000 -> 500)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t lastHeapCheck = 0;
    uint32_t lastBatteryRead = 0;
    uint32_t lastBLEDataSend = 0;

    // Локальные буферы для BLE данных
    // char local_ble_command[32];
    // char local_ble_address[32];
    bool local_ble_connected;
    char local_filename[64];
    float local_battery_voltage;
    uint8_t local_battery_percent;
    unsigned long local_recording_start_time;
    unsigned long local_recording_duration;
    bool local_recording_is_run;
    float local_left_angle = 0;
    float local_right_angle = 0;
    ODriveErrors local_left_errors = {0};
    ODriveErrors local_right_errors = {0};

    // static uint8_t last_sent_battery = 255;
    // static String last_sent_time = "";
    // static uint32_t last_battery_notify = 0;
    // static uint32_t last_time_notify = 0;

    // Инициализация буферов
    memset(local_filename, 0, sizeof(local_filename));
    
    for (;;) {
        xTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Чтение напряжения батареи (раз в 5 секунд)
        if (millis() - lastBatteryRead >= 5000) {
            int raw_value = analogRead(BATTERY_ADC_PIN);
            battery_voltage = (float)raw_value / 4095.0 * 3.3 * 10 * 1.19;
            battery_percent = calculateBatteryPercent(battery_voltage);
            lastBatteryRead = millis();
        }

        // Логирование состояния системы
        if (millis() - lastHeapCheck >= 5000) {
            int queueSize = uxQueueMessagesWaiting(dataPairQueue);

            if (xSemaphoreTake(xBLEMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                local_ble_connected = ble_connected;
                strncpy(local_filename, (char*)current_filename, 63);
                local_filename[63] = '\0';
                local_recording_start_time = recording_start_time;
                local_recording_duration = recording_duration;
                local_recording_is_run = recording_is_run;
                xSemaphoreGive(xBLEMutex);
            }

            // Получаем BLE статус с защитой мьютексом
            // if (xSemaphoreTake(xBLEMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            //     local_ble_connected = ble_connected;
            //     strncpy(local_ble_command, (char*)ble_last_command, 31);
            //     local_ble_command[31] = '\0';
            //     strncpy(local_ble_address, (char*)ble_client_address, 31);
            //     local_ble_address[31] = '\0';
            //     xSemaphoreGive(xBLEMutex);
            // }

            Serial.printf("System: Heap=%d, CAN Queue=%d/%d, Battery=%.2fV (%d%%), BLE=%s\n",
                         ESP.getFreeHeap(),
                         queueSize, DATA_PAIR_QUEUE_SIZE,
                         battery_voltage, battery_percent,
                         local_ble_connected ? "CONNECTED" : "DISCONNECTED");
            
            Serial.printf("Recording: %s, Duration: %s, File: %s\n",
                         local_recording_is_run ? "ON" : "OFF",
                         formatTime(local_recording_duration).c_str(),
                         local_filename);
            lastHeapCheck = millis();
        }

        // Сбор текущих данных с защитой мьютексом
        if (xSemaphoreTake(xBLEMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            local_ble_connected = ble_connected;
            strncpy(local_filename, (char*)current_filename, 63);
            local_filename[63] = '\0';
            local_recording_start_time = recording_start_time;
            local_recording_duration = recording_duration;
            local_recording_is_run = recording_is_run;
            xSemaphoreGive(xBLEMutex);
        }

        // Получаем данные моторов
        if (xSemaphoreTake(xOdriveMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            local_left_angle = MOTOR_ANGLE_TO_DEG(axes[1].encoder_estimates.position);
            local_right_angle = -MOTOR_ANGLE_TO_DEG(axes[0].encoder_estimates.position);

            if (axes[1].last_errors.axis_error)
                local_left_errors = axes[1].last_errors;
            if (axes[0].last_errors.axis_error)
                local_right_errors = axes[0].last_errors;
            xSemaphoreGive(xOdriveMutex);
        }
        
        local_battery_voltage = battery_voltage;
        local_battery_percent = battery_percent;
        
        // Если запись активна, обновляем текущую длительность
        if (local_recording_is_run) {
            local_recording_duration = millis() - local_recording_start_time;
        }

        // Отправка данных по BLE (каждые 500 мс)
        if (local_ble_connected && pDataCharacteristic) {
            // Создаем JSON документ
            JsonDocument doc;
            
            // Добавляем данные
            doc["left_angle"] = local_left_angle;
            doc["right_angle"] = local_right_angle;
            doc["battery_voltage"] = local_battery_voltage;
            doc["battery_percent"] = local_battery_percent;
            doc["filename"] = local_filename;
            doc["recording"] = local_recording_is_run;
            doc["recording_duration"] = local_recording_duration;
            
            // Сериализуем JSON в строку
            char jsonBuffer[512];
            size_t len = serializeJson(doc, jsonBuffer);
            
            // Отправляем по BLE
            pDataCharacteristic->setValue(String(jsonBuffer));
            pDataCharacteristic->notify();
            
            // Отладка (раз в 5 секунд)
            if (millis() - lastBLEDataSend > 5000) {
                Serial.printf("BLE: Sent data packet (%d bytes)\n", len);
                serializeJsonPretty(doc, Serial);
                Serial.println();
                lastBLEDataSend = millis();
            }
        }
        
        // // NEW ---------------------------------------------------------------------------------
        // // Отправка уровня заряда батареи (каждые 5 секунд или при изменении)
        // if (local_ble_connected && pBatteryCharacteristic) {
        //     if (local_battery_percent != last_sent_battery || (millis() - last_battery_notify) >= 5000) {
        //         char battery_str[32];
        //         snprintf(battery_str, sizeof(battery_str), "Battery=%d", local_battery_percent);
        //         pBatteryCharacteristic->setValue(battery_str);
        //         pBatteryCharacteristic->notify();
        //         last_sent_battery = local_battery_percent;
        //         last_battery_notify = millis();
                
        //         if (millis() - lastBLEDataSend > 5000) {
        //             Serial.printf("BLE: Sent battery update: %s\n", battery_str);
        //         }
        //     }
        // }

        // // Отправка времени записи (каждую секунду при записи)
        // if (local_ble_connected && pTimeCharacteristic) {
        //     bool should_send_time = false;
            
        //     if (local_recording_is_run) {
        //         // При активной записи отправляем каждую секунду
        //         if (millis() - last_time_notify >= 1000) {
        //             should_send_time = true;
        //         }
        //     } else if (last_sent_time != "Time=00:00") {
        //         // При остановке записи отправляем 00:00 один раз
        //         should_send_time = true;
        //     }
            
        //     if (should_send_time) {
        //         char time_str[32];
        //         String formatted_time = formatTime(local_recording_duration);
        //         snprintf(time_str, sizeof(time_str), "Time=%s", formatted_time.c_str());
                
        //         // Отправляем только если значение изменилось
        //         if (String(time_str) != last_sent_time) {
        //             pTimeCharacteristic->setValue(time_str);
        //             pTimeCharacteristic->notify();
        //             last_sent_time = time_str;
        //             last_time_notify = millis();
                    
        //             if (millis() - lastBLEDataSend > 5000) {
        //                 Serial.printf("BLE: Sent time update: %s\n", time_str);
        //             }
        //         }
                
        //         // Если запись остановлена, сбрасываем для следующей отправки при старте
        //         if (!local_recording_is_run && formatted_time == "00:00") {
        //             last_sent_time = "Time=00:00";
        //         }
        //     }
        // }
        // // NEW ---------------------------------------------------------------------------------

        // Обновление дисплея
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        
        if (local_left_errors.axis_error || local_right_errors.axis_error) {
            display.setCursor(0, 0);
            display.printf("L:A:%xM:%xE:%xC:%x", local_left_errors.axis_error, local_left_errors.motor_error, local_left_errors.encoder_error, local_left_errors.controller_error);
            display.setCursor(0, 24);
            display.printf("R:A:%xM:%xE:%xC:%x", local_right_errors.axis_error, local_right_errors.motor_error, local_right_errors.encoder_error, local_right_errors.controller_error);
        } else {
            // Строка 1: Углы моторов
            display.setCursor(0, 0);
            float display_left, display_right;
            int queueSize = uxQueueMessagesWaiting(dataPairQueue);
            
            
            display.printf("L:%.1f R:%.1f", display_left, display_right);
            
            // Строка 2: Напряжение батареи и процент
            display.setCursor(0, 12);
            display.printf("Batt:%.2fV", battery_voltage);
            display.setCursor(72, 12);
            display.printf("Bat:%d%%", local_battery_percent);
            
            // Строка 3: Статус записи и очередь
            display.setCursor(0, 24);
            display.printf("Rec:%s Q:%d/%d", recording_is_run ? "ON" : "OFF", queueSize, DATA_PAIR_QUEUE_SIZE);
            
            // Строка 4: BLE статус и длительность записи
            display.setCursor(0, 36);
            
            // Получаем BLE статус с защитой мьютексом
            // if (xSemaphoreTake(xBLEMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            //     local_ble_connected = ble_connected;
            //     strncpy(local_ble_address, (char*)ble_client_address, 31);
            //     local_ble_address[31] = '\0';
            //     strncpy(local_ble_command, (char*)ble_last_command, 31);
            //     local_ble_command[31] = '\0';
            //     xSemaphoreGive(xBLEMutex);
            // }
            
            display.printf("BLE:%s", local_ble_connected ? "CON" : "DIS");
    
            display.setCursor(48, 36);
            if (local_recording_is_run || local_recording_duration > 0) {
                display.printf("Time:%s", formatTime(local_recording_duration).c_str());
            } else {
                display.print("Time:00:00");
            }
            
            // Строка 5: Имя файла (обрезаем если слишком длинное)
            display.setCursor(0, 48);
    
            if (strlen(local_filename) > 0) {
                // Отображаем только имя файла без расширения, обрезаем до 16 символов
                char short_name[17] = {0};
                char* dot_pos = strchr(local_filename, '.');
                if (dot_pos != NULL) {
                    int len = min((int)(dot_pos - local_filename), 16);
                    strncpy(short_name, local_filename, len);
                    short_name[len] = '\0';
                } else {
                    strncpy(short_name, local_filename, 16);
                    short_name[16] = '\0';
                }
                display.printf("File:%s", short_name);
            } else {
                display.print("File:---");
            }
        }
        
        display.display();
    }
}

#endif