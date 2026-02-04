#ifndef CAN_RECEIVER_H
#define CAN_RECEIVER_H

#include <Arduino.h>
#include <driver/twai.h>
#include "../config.h"
#include "../globals.h"
#include "../utils/can_utils.h"
#include "odrive/utils/odrive_can.h"

// Задача для приема CAN сообщений с использованием прерываний
void taskCANReceiver(void *pvParameters)
{
    twai_message_t message;
    uint32_t last_debug = 0;

    while (1)
    {
        // Постоянно опрашиваем CAN на наличие сообщений
        if (twai_receive(&message, pdMS_TO_TICKS(1)) == ESP_OK)
        {
            if (xSemaphoreTake(xOdriveMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                odrive_update(axes, AXIS_COUNT, message);
                xSemaphoreGive(xOdriveMutex);
            }

            // if (message.data_length_code == 4) {
            //     float angle = bytesToFloat(message.data);
            //     uint32_t timestamp = millis();

            // Отладочный вывод
            if (millis() - last_debug > 2000)
            {
                Serial.printf("CAN Received: ID=0x%03X\n", message.identifier);
                last_debug = millis();
            }

            //     // Защищенное обновление буферов
            //     if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            //         if (message.identifier == LEFT_MOTOR_ID) {
            //             left_motor.angle = angle;
            //             left_motor.timestamp = timestamp;
            //             left_motor.valid = true;
            //         } else if (message.identifier == RIGHT_MOTOR_ID) {
            //             right_motor.angle = angle;
            //             right_motor.timestamp = timestamp;
            //             right_motor.valid = true;
            //         }
            //         xSemaphoreGive(xDataMutex);
            //     }
            // }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

#endif