#include <Arduino.h>
#include "config.h"
#include "globals.h"

// Подключение заголовочных файлов задач
#include "tasks/can_receiver.h"
#include "tasks/data_pairing.h"
#include "tasks/sd_writer.h"
#include "tasks/ui_handler.h"
#include "tasks/system_monitor.h"
#include "tasks/ble_server.h"

// Подключение утилит
#include "utils/pins.h"
#include "utils/can_utils.h"
#include "utils/sd_utils.h"
#include "utils/display_utils.h"
#include "utils/recording.h"

// Одрайв
#include "odrive/tasks/motor_control.h"

// Определение глобальных переменных
volatile bool recording_is_run = false;
volatile bool server_recording = false;
volatile float battery_voltage = 0.0;
volatile uint8_t battery_percent = 0;

// BLE переменные
volatile bool ble_connected = false;
volatile char ble_last_command[32] = {0};
volatile char ble_client_address[32] = {0};

// Переменные для записи времени
volatile unsigned long recording_start_time = 0;
volatile unsigned long recording_duration = 0;
volatile char current_filename[64] = {0};

// Переменные для настроек GAIN и DELAY
volatile int gain_setting = GAIN;
volatile uint32_t delay_setting = DELAY;

// Глобальные буферы для данных двигателей
imu_buffer_t left_imu = {0, 0, false};
imu_buffer_t right_imu = {0, 0, false};

// Определение глобальных объектов
Adafruit_SSD1306 display(128, 64, &Wire, -1);
GButton buttonStart(BTN_START_PIN);
GButton buttonStop(BTN_STOP_PIN);
File dataFile;

// BLE объекты
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pRecordControlCharacteristic = nullptr;
NimBLECharacteristic* pDataCharacteristic = nullptr;

NimBLECharacteristic* pBatteryCharacteristic = nullptr;
NimBLECharacteristic* pTimeCharacteristic = nullptr;

NimBLECharacteristic* pSettingsCharacteristic = nullptr; // GAIN DELAY

// Определение очередей и семафоров
QueueHandle_t dataPairQueue;
SemaphoreHandle_t xDataMutex;
SemaphoreHandle_t xSDMutex;
SemaphoreHandle_t xRecordMutex;
SemaphoreHandle_t xBLEMutex;
SemaphoreHandle_t xOdriveMutex;

SemaphoreHandle_t xSettingsMutex;  // GAIN DELAY

// Управление моторчиком
ODriveAxis axes[AXIS_COUNT] = {{.state_transition_callback = odrive_state_transition_callback,
                                .state_transition_context = NULL},
                               {.state_transition_callback = odrive_state_transition_callback,
                                .state_transition_context = NULL},
                              {.state_transition_callback = odrive_state_transition_callback,
                                .state_transition_context = NULL},
                               {.state_transition_callback = odrive_state_transition_callback,
                                .state_transition_context = NULL}};
stamp_tracker tracker = {0};

void setup() {
  Serial.begin(115200);
  // delay(3000);
  Serial.println("ESP32-TWAI FreeRTOS Starting...");
  
  initPins();
  
  if (!initDisplay()) {
    Serial.println("Display initialization failed!");
  }
  
  if (!initSDCard()) {
    Serial.println("SD card initialization failed!");
  }
  
  if (!initCAN()) {
    Serial.println("CAN initialization failed!");
  }

  // Инициализация очередей и семафоров
  dataPairQueue = xQueueCreate(DATA_PAIR_QUEUE_SIZE, sizeof(paired_data_t));
  xDataMutex = xSemaphoreCreateMutex();
  xSDMutex = xSemaphoreCreateMutex();
  xRecordMutex = xSemaphoreCreateMutex();
  xBLEMutex = xSemaphoreCreateMutex();
  xOdriveMutex = xSemaphoreCreateMutex();

  xSettingsMutex = xSemaphoreCreateMutex();  // GAIN DELAY

  // Проверка создания объектов FreeRTOS
  if (dataPairQueue == NULL || 
      xDataMutex == NULL || 
      xSDMutex == NULL || 
      xRecordMutex == NULL || 
      xBLEMutex == NULL || 
      xOdriveMutex == NULL || 
      xSettingsMutex == NULL) 
  {
    Serial.println("ERROR: Failed to create FreeRTOS objects!");
    while(1) delay(1000);
  }

  // Создание задач
  xTaskCreate(taskMotorControl, "MotorController", 8192, NULL, 9, NULL);// Самый убер мега высокий приоритет
  xTaskCreate(taskCANReceiver, "CANReceiver", 8192, NULL, 8, NULL);     // Самый высокий приоритет
  xTaskCreate(taskDataPairing, "DataPairing", 8192, NULL, 7, NULL);     // Высокий приоритет
  xTaskCreate(taskSDWriter, "SDWriter", 12288, NULL, 6, NULL);          // Высокий приоритет 
  xTaskCreate(taskBLEServer, "BLEServer", 8192, NULL, 5, NULL);         // BLE сервер (средний приоритет)
  xTaskCreate(taskUIHandler, "UIHandler", 4096, NULL, 2, NULL);         // Низкий приоритет
  xTaskCreate(taskSystemMonitor, "SystemMonitor", 8192, NULL, 1, NULL); // Самый низкий

  Serial.println("FreeRTOS tasks started successfully");
  vTaskDelete(NULL);
}

void loop() {
  vTaskDelete(NULL);
}