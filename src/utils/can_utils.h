#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include <Arduino.h>
#include <driver/twai.h>
#include "../config.h"
#include "../globals.h"

void debugCANStatus();

// Функция для преобразования байт в float
float bytesToFloat(const byte *bytes_array) {
  union {
    float float_variable;
    byte byte_array[4];
  } data;
  memcpy(data.byte_array, bytes_array, 4);
  return data.float_variable;
}

// Инициализация CAN шины
bool initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Настройка для максимальной производительности
    g_config.rx_queue_len = 10;
    g_config.tx_queue_len = 10;
    
    // Упрощенная настройка фильтра - принимаем все сообщения для отладки
    f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result != ESP_OK) {
        Serial.printf("Failed to install TWAI driver: 0x%04X\n", result);
        return false;
    }

    result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("Failed to start TWAI driver: 0x%04X\n", result);
        return false;
    }
    
    Serial.println("TWAI (CAN) initialized successfully");
    
    // Вывод статуса CAN для отладки
    debugCANStatus();
    
    return true;
}

// Дебаг CAN
void debugCANStatus() {
    twai_status_info_t status_info;
    twai_get_status_info(&status_info);
    
    Serial.println("=== CAN Status ===");
    Serial.printf("State: %d\n", status_info.state);
    Serial.printf("Msgs to RX: %d\n", status_info.msgs_to_rx);
    Serial.printf("Msgs to TX: %d\n", status_info.msgs_to_tx);
    Serial.printf("TX Error Counter: %d\n", status_info.tx_error_counter);
    Serial.printf("RX Error Counter: %d\n", status_info.rx_error_counter);
    Serial.printf("TX Failed: %d\n", status_info.tx_failed_count);
    Serial.printf("RX Missed: %d\n", status_info.rx_missed_count);
    Serial.printf("RX Overrun: %d\n", status_info.rx_overrun_count);
    Serial.println("===================");
}

#endif