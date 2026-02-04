#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string.h>  // Для strncpy
#include "../config.h"
#include "../globals.h"
#include "../utils/recording.h"

/** Callbacks сервера BLE */
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        Serial.printf("BLE Client connected: %s\n", connInfo.getAddress().toString().c_str());
        
        // Обновляем данные для дисплея
        if (xSemaphoreTake(xBLEMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ble_connected = true;
            // Используем strncpy для копирования строки
            strncpy((char*)ble_client_address, connInfo.getAddress().toString().c_str(), 31);
            ble_client_address[31] = '\0';  // Гарантируем нулевое окончание
            xSemaphoreGive(xBLEMutex);
        }

        // Устанавливаем параметры соединения
        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.printf("BLE Client disconnected - start advertising\n");

        // Обновляем данные для дисплея
        if (xSemaphoreTake(xBLEMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ble_connected = false;
            ble_client_address[0] = '\0';  // Очищаем строку
            xSemaphoreGive(xBLEMutex);
        }
        
        // Перезапускаем рекламу
        NimBLEDevice::startAdvertising();
    }
};

/** Handler для характеристики управления записью */
class RecordControlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        String command = pCharacteristic->getValue().c_str();
        command.toLowerCase(); // Приводим к нижнему регистру для удобства сравнения
        Serial.printf("BLE Command received: %s\n", command.c_str());
        
        // Обновляем данные для дисплея
        if (xSemaphoreTake(xBLEMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Используем strncpy для копирования строки
            strncpy((char*)ble_last_command, command.c_str(), 31);
            ble_last_command[31] = '\0';  // Гарантируем нулевое окончание
            xSemaphoreGive(xBLEMutex);
        }
        
        // Обрабатываем команды
        if (command == "start" || command == "start recording" || command == "1") {
            Serial.println("BLE: Starting recording...");
            startRecording();
        } 
        else if (command == "stop" || command == "stop recording" || command == "0") {
            Serial.println("BLE: Stopping recording...");
            stopRecording();
        }
        else {
            Serial.printf("BLE: Unknown command: %s\n", command.c_str());
        }
        
        // Отправляем подтверждение обратно клиенту
        String response = "CMD: " + command + " - ";
        response += recording_is_run ? "Recording ACTIVE" : "Recording STOPPED";
        pCharacteristic->setValue(response.c_str());
        pCharacteristic->notify();
    }

    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
        Serial.printf("BLE Client subscribed to record control\n");
    }
};

// Колбэки для настроек
class SettingsCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        String command = pCharacteristic->getValue().c_str();
        command.trim();
        
        Serial.printf("BLE Settings received: %s\n", command.c_str());
        
        // Обновляем данные для дисплея (опционально)
        if (xSemaphoreTake(xBLEMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            strncpy((char*)ble_last_command, command.c_str(), 31);
            ble_last_command[31] = '\0';
            xSemaphoreGive(xBLEMutex);
        }
        
        // Обрабатываем команды для GAIN и DELAY
        bool validCommand = false;
        String response = "ERROR: Invalid format";
        
        // Проверяем формат "GAIN=xxx" или "DELAY=xxx"
        int equalPos = command.indexOf('=');
        if (equalPos > 0) {
            String param = command.substring(0, equalPos);
            param.toUpperCase();
            String valueStr = command.substring(equalPos + 1);
            
            if (param == "GAIN") {
                int newGain = valueStr.toInt();
                if (newGain >= MIN_GAIN && newGain <= MAX_GAIN) { // пределы
                    if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        gain_setting = newGain;
                        xSemaphoreGive(xSettingsMutex);
                        validCommand = true;
                        response = "GAIN set to " + valueStr;
                        Serial.printf("GAIN updated to: %d\n", newGain);
                    }
                } else {
                    response = "ERROR: GAIN must be between " + String(MIN_GAIN) + " and " + String(MAX_GAIN);
                }
            } 
            else if (param == "DELAY") {
                uint32_t newDelay = valueStr.toInt();
                if (newDelay >= MIN_DELAY && newDelay <= MAX_DELAY) { // пределы
                    if (xSemaphoreTake(xSettingsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        delay_setting = newDelay * 2;
                        xSemaphoreGive(xSettingsMutex);
                        validCommand = true;
                        response = "DELAY set to " + valueStr + " ms";
                        Serial.printf("DELAY updated to: %lu ms\n", newDelay);
                    }
                } else {
                    response = "ERROR: DELAY must be between " + String(MIN_DELAY) + " and " + String(MAX_DELAY) + " ms";
                }
            }
        }
        
        // Отправляем ответ обратно клиенту
        pCharacteristic->setValue(response.c_str());
        pCharacteristic->notify();
    }
    
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
        Serial.printf("BLE Client subscribed to settings\n");
    }
};

// Задача для BLE сервера
void taskBLEServer(void *pvParameters) {
    Serial.println("Starting BLE Server...");

    // Инициализация BLE
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Максимальная мощность передачи
    
    // Создаем сервер и устанавливаем колбэки
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // Создаем сервис для управления записью
    NimBLEService* pRecordService = pServer->createService(BLE_SERVICE_UUID);
    
    // Создаем характеристику для управления записью
    pRecordControlCharacteristic = pRecordService->createCharacteristic(
        BLE_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | 
        NIMBLE_PROPERTY::WRITE |
        NIMBLE_PROPERTY::WRITE_NR |
        NIMBLE_PROPERTY::NOTIFY
    );

    pRecordControlCharacteristic->setValue("Ready for commands");
    pRecordControlCharacteristic->setCallbacks(new RecordControlCallbacks());

    // Создаем характеристику для передачи данных
    pDataCharacteristic = pRecordService->createCharacteristic(
        BLE_DATA_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | 
        NIMBLE_PROPERTY::NOTIFY
    );

    pDataCharacteristic->setValue("System Data");

    // NEW ---------------------------------------------------------------------------------
    // Создаем характеристику для уровня заряда батареи
    pBatteryCharacteristic = pRecordService->createCharacteristic(
        BLE_BATTERY_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | 
        NIMBLE_PROPERTY::NOTIFY
    );
    pBatteryCharacteristic->setValue("Battery=0");

    // Создаем характеристику для времени записи
    pTimeCharacteristic = pRecordService->createCharacteristic(
        BLE_TIME_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | 
        NIMBLE_PROPERTY::NOTIFY
    );
    pTimeCharacteristic->setValue("Time=00:00");
    // NEW ---------------------------------------------------------------------------------

     // Создаем характеристику для настроек GAIN и DELAY
    pSettingsCharacteristic = pRecordService->createCharacteristic(
        BLE_SETTINGS_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | 
        NIMBLE_PROPERTY::WRITE |
        NIMBLE_PROPERTY::WRITE_NR |
        NIMBLE_PROPERTY::NOTIFY
    );

    pSettingsCharacteristic->setValue("GAIN=30;DELAY=30");
    pSettingsCharacteristic->setCallbacks(new SettingsCallbacks());

    // Запускаем сервис
    pRecordService->start();

    // Настраиваем рекламу
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName(BLE_DEVICE_NAME);
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setAppearance(0x0000); // Generic appearance
    
    // Устанавливаем параметры рекламы
    pAdvertising->setMinInterval(400);   // 400 * 0.625ms = 250ms
    pAdvertising->setMaxInterval(800);   // 800 * 0.625ms = 500ms
    pAdvertising->start();

    Serial.printf("BLE Server Started: %s\n", BLE_DEVICE_NAME);
    Serial.printf("Service UUID: %s\n", BLE_SERVICE_UUID);
    Serial.printf("Characteristic UUID: %s\n", BLE_CHARACTERISTIC_UUID);

    // NEW ---------------------------------------------------------------------------------
    // Serial.printf("Battery Characteristic UUID: %s\n", BLE_BATTERY_CHARACTERISTIC_UUID);
    // Serial.printf("Time Characteristic UUID: %s\n", BLE_TIME_CHARACTERISTIC_UUID);
    // NEW ---------------------------------------------------------------------------------
    Serial.printf("Settings Characteristic UUID: %s\n", BLE_SETTINGS_CHARACTERISTIC_UUID);
    
    Serial.println("Send 'start' or 'stop' commands to control recording");
    
    // Основной цикл задачи BLE
    while (1) {
        // Просто поддерживаем соединение, периодических уведомлений не отправляем
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif