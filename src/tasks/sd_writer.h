#ifndef SD_WRITER_H
#define SD_WRITER_H

#include <Arduino.h>
#include <SD.h>
#include <math.h>
#include "../config.h"
#include "../globals.h"

// Структура для тестовых данных синусоиды
typedef struct {
    uint32_t timestamp;
    float left_angle;
    float right_angle;
    bool is_test_data;
} data_record_t;

// Задача для записи данных на SD-карту с буферизацией
void taskSDWriter(void *pvParameters) {
    data_record_t writeBuffer[WRITE_BUFFER_SIZE];
    uint16_t bufferIndex = 0;
    uint32_t lastFlushTime = millis();
    uint32_t lastBlinkTime = 0;
    uint32_t lastDebugTime = 0;
    uint32_t writeCount = 0;
    uint32_t lastRealDataTime = 0;
    bool ledState = false;
    bool testDataMode = false;
    
    // Переменные для генерации синусоиды
    uint32_t testDataStartTime = 0;
    uint32_t lastTestSampleTime = 0;
    const float FREQUENCY = 0.5f; // Гц
    const float AMPLITUDE = 90.0f; // Амплитуда в градусах
    const float LEFT_PHASE = 0.0f; // Фаза для левого мотора
    const float RIGHT_PHASE = M_PI / 2.0f; // Сдвиг фазы на 90 градусов для правого мотора
    
    // Настройки дискретизации
    const uint32_t SAMPLING_INTERVAL = 5; // Интервал дискретизации в мс (50 Гц)
    const uint32_t TEST_DATA_TIMEOUT = 500; // Таймаут для перехода в тестовый режим (мс)
    
    while (1) {
        data_record_t data;
        bool hasData = false;
        
        // Сначала проверяем очередь реальных данных без таймаута (non-blocking)
        if (xQueueReceive(dataPairQueue, &data, 0) == pdTRUE) {
            data.is_test_data = false; // Это реальные данные
            hasData = true;
            lastRealDataTime = millis();
            testDataMode = false; // При получении реальных данных выключаем тестовый режим
            testDataStartTime = 0; // Сбрасываем генератор тестовых данных
            
            writeCount++;
            
            // Отладочный вывод каждые 100 записей реальных данных
            // if (writeCount % 100 == 0) {
            //     Serial.printf("SDWriter: Real data #%lu, L=%.1f, R=%.1f\n", 
            //                  writeCount, data.left_angle, data.right_angle);
            // }
        } 
        // Если нет реальных данных более TEST_DATA_TIMEOUT, переходим в тестовый режим
        // else if (recording_is_run && (millis() - lastRealDataTime > TEST_DATA_TIMEOUT)) {
        //     testDataMode = true;
            
        //     // Генерация тестовых данных синусоиды с фиксированной частотой дискретизации
        //     uint32_t currentTime = millis();
            
        //     // Проверяем, пришло ли время для нового сэмпла
        //     if (currentTime - lastTestSampleTime >= SAMPLING_INTERVAL) {
        //         lastTestSampleTime = currentTime;
                
        //         if (testDataStartTime == 0) {
        //             testDataStartTime = currentTime;
        //         }
                
        //         uint32_t elapsedTime = currentTime - testDataStartTime;
        //         float timeInSeconds = elapsedTime / 1000.0f;
                
        //         // Генерация синусоидальных углов
        //         data.timestamp = currentTime;
        //         data.left_angle = AMPLITUDE * sin(2 * M_PI * FREQUENCY * timeInSeconds + LEFT_PHASE);
        //         data.right_angle = AMPLITUDE * sin(2 * M_PI * FREQUENCY * timeInSeconds + RIGHT_PHASE);
        //         data.is_test_data = true; // Это тестовые данные
        //         hasData = true;
                
        //         // Отладочный вывод каждые 2 секунды в тестовом режиме
        //         // if (currentTime - lastDebugTime > 2000) {
        //         //     // Serial.printf("SDWriter: Test data @%lu - L=%.1f, R=%.1f (freq=%.1fHz)\n", 
        //         //     //              currentTime, data.left_angle, data.right_angle, 
        //         //     //              1000.0/SAMPLING_INTERVAL);
        //         //     lastDebugTime = currentTime;
        //         // }
        //     }
        // }
        
        if (hasData && recording_is_run) {
            if (!dataFile) {
                Serial.println("SDWriter: dataFile is null!");
                continue;
            }
            
            // Добавляем в буфер
            if (bufferIndex < WRITE_BUFFER_SIZE) {
                writeBuffer[bufferIndex++] = data;
            } else {
                Serial.println("SDWriter: Write buffer overflow!");
            }
            
            // Если буфер заполнен или прошло время - записываем
            if (bufferIndex >= WRITE_BUFFER_SIZE || (millis() - lastFlushTime) > 1000) {
                if (bufferIndex > 0) {
                    if (xSemaphoreTake(xSDMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        // Статистика перед записью
                        uint16_t testCount = 0;
                        uint16_t realCount = 0;
                        for (uint16_t i = 0; i < bufferIndex; i++) {
                            if (writeBuffer[i].is_test_data) testCount++;
                            else realCount++;
                        }
                        
                        // Serial.printf("SDWriter: Flushing %d records (%d real, %d test) to SD\n", 
                        //              bufferIndex, realCount, testCount);
                        
                        // Записываем весь буфер
                        for (uint16_t i = 0; i < bufferIndex; i++) {
                            // Добавляем метку [TEST] если это тестовые данные
                            if (writeBuffer[i].is_test_data) {
                                dataFile.printf("%lu,%.4f,%.4f,[TEST]\n", 
                                              writeBuffer[i].timestamp,
                                              writeBuffer[i].left_angle, 
                                              writeBuffer[i].right_angle);
                            } else {
                                dataFile.printf("%lu,%.4f,%.4f\n", 
                                              writeBuffer[i].timestamp,
                                              writeBuffer[i].left_angle, 
                                              writeBuffer[i].right_angle);
                            }
                        }
                        
                        dataFile.flush();
                        bufferIndex = 0;
                        lastFlushTime = millis();
                        xSemaphoreGive(xSDMutex);
                        
                        Serial.printf("SDWriter: Successfully flushed at %lu\n", lastFlushTime);
                    } else {
                        Serial.println("SDWriter: Failed to take SD mutex!");
                    }
                }
            }
            
            // Мигание LED при записи
            if (millis() - lastBlinkTime >= 200) {
                lastBlinkTime = millis();
                digitalWrite(LED_REC_PIN, ledState);
                ledState = !ledState;
                
                // Дополнительная индикация для тестового режима
                // if (testDataMode) {
                //     digitalWrite(LED_OK_PIN, ledState); // Мигаем вторым светодиодом в тестовом режиме
                // } else {
                //     digitalWrite(LED_OK_PIN, LOW); // Выключаем второй светодиод в режиме реальных данных
                // }
            }
        }
        
        // Принудительный flush при остановке записи
        if (!recording_is_run && bufferIndex > 0) {
            Serial.printf("SDWriter: Final flush of %d records before stop\n", bufferIndex);
            if (xSemaphoreTake(xSDMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (dataFile) {
                    for (uint16_t i = 0; i < bufferIndex; i++) {
                        if (writeBuffer[i].is_test_data) {
                            dataFile.printf("%lu,%.4f,%.4f,[TEST]\n", 
                                          writeBuffer[i].timestamp,
                                          writeBuffer[i].left_angle, 
                                          writeBuffer[i].right_angle);
                        } else {
                            dataFile.printf("%lu,%.4f,%.4f\n", 
                                          writeBuffer[i].timestamp,
                                          writeBuffer[i].left_angle, 
                                          writeBuffer[i].right_angle);
                        }
                    }
                    dataFile.flush();
                    Serial.println("SDWriter: Final flush completed");
                }
                bufferIndex = 0;
                testDataMode = false;
                testDataStartTime = 0;
                lastTestSampleTime = 0;
                xSemaphoreGive(xSDMutex);
            }
        }
        
        // Периодическая проверка состояния
        if (millis() - lastDebugTime > 10000) {
            int queueSize = uxQueueMessagesWaiting(dataPairQueue);
            
            // Вычисляем фактическую частоту дискретизации
            float samplingFreq = 0;
            if (testDataMode) {
                samplingFreq = 1000.0 / SAMPLING_INTERVAL;
            }
            
            // Serial.printf("SDWriter Status: Recording=%s, TestMode=%s, Freq=%.1fHz, Buffer=%d/%d, Queue=%d, File=%s\n",
            //              recording_is_run ? "YES" : "NO",
            //              testDataMode ? "YES" : "NO",
            //              samplingFreq,
            //              bufferIndex, WRITE_BUFFER_SIZE,
            //              queueSize,
            //              dataFile ? "OPEN" : "CLOSED");
            lastDebugTime = millis();
        }

        if (recording_is_run) {
            // Обновляем длительность записи
            if (xSemaphoreTake(xBLEMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                recording_duration = millis() - recording_start_time;
                xSemaphoreGive(xBLEMutex);
            }
        }
        
        // Короткая задержка для предотвращения блокировки задачи
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

#endif