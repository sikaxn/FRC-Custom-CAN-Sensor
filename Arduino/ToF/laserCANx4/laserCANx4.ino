#include <Wire.h>
#include <VL53L1X.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/twai.h"

#define SDA_PIN 21
#define SCL_PIN 22
#define NUM_SENSORS 4

#define DEVICE_ID        0x0A
#define MANUFACTURER_ID  0x08
#define DEVICE_NUMBER    50
#define SENSOR_BASE_API_ID 0x0301
#define SENSOR_CONFIG_API_ID 0x0305

VL53L1X sensors[NUM_SENSORS];
int sensorDistances[NUM_SENSORS];

const uint8_t xshutPins[NUM_SENSORS] = {16, 17, 18, 19};
const uint8_t sensorAddresses[NUM_SENSORS] = {0x30, 0x31, 0x32, 0x33};
const uint8_t roiCenters[NUM_SENSORS] = {199, 199, 199, 199};
const uint32_t timingBudgets[NUM_SENSORS] = {50000, 50000, 50000, 50000};
const VL53L1X::DistanceMode rangingModes[NUM_SENSORS] = {
  VL53L1X::Long,
  VL53L1X::Long,
  VL53L1X::Long,
  VL53L1X::Long
};

SemaphoreHandle_t sensorMutex;

uint32_t makeCANMsgID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID) << 24) |
         ((uint32_t)(manufacturerID) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) |
         (deviceNumber & 0x3F);
}

void TaskSensorRead(void* pvParams) {
  for (;;) {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    for (int i = 0; i < NUM_SENSORS; i++) {
      int dist = sensors[i].read();
      if (sensors[i].timeoutOccurred() || dist <= 0 || dist > 4000) {
          dist = 0;
          }
      sensorDistances[i] = dist;

    }
    xSemaphoreGive(sensorMutex);
    vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz
  }
}

void TaskSensorPrint(void* pvParams) {
  for (;;) {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(sensorDistances[i]);
      if (i < NUM_SENSORS - 1) Serial.print('\t');
    }
    Serial.println();
    xSemaphoreGive(sensorMutex);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void TaskCANTx(void* pvParams) {
  for (;;) {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    for (int i = 0; i < NUM_SENSORS; i++) {
      int dist = sensorDistances[i];
      uint32_t tb = sensors[i].getMeasurementTimingBudget();

      twai_message_t msg = {};
      msg.identifier = makeCANMsgID(DEVICE_ID, MANUFACTURER_ID, SENSOR_BASE_API_ID + i, DEVICE_NUMBER);
      msg.extd = 1;
      msg.data_length_code = 8;

      msg.data[0] = (dist >> 8) & 0xFF;         // Distance high byte
      msg.data[1] = dist & 0xFF;                // Distance low byte
      msg.data[2] = sensors[i].getDistanceMode(); // Ranging mode
      msg.data[3] = tb >> 24;
      msg.data[4] = tb >> 16;
      msg.data[5] = tb >> 8;
      msg.data[6] = tb;
      msg.data[7] = sensors[i].getROICenter();   // ROI center

      twai_transmit(&msg, pdMS_TO_TICKS(10));
    }
    xSemaphoreGive(sensorMutex);
    vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
  }
}

void TaskCANRx(void* pvParams) {
  for (;;) {
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(10)) != ESP_OK) continue;

    if (!msg.extd || msg.data_length_code < 2) continue;

    uint16_t apiId = (msg.identifier >> 6) & 0x3FF;
    uint8_t deviceNum = msg.identifier & 0x3F;
    if (deviceNum != DEVICE_NUMBER) continue;

    for (int i = 0; i < NUM_SENSORS; i++) {
      if (apiId == SENSOR_CONFIG_API_ID + i) {
        uint8_t newMode = msg.data[0];
        uint8_t newRoi  = msg.data[1];

        if (newMode > 2) {
          Serial.printf("[CAN RX] Invalid mode (%d) for sensor %d, ignoring.\n", newMode, i);
          break;
        }

        if (newRoi > 255) {
          Serial.printf("[CAN RX] Invalid ROI center (%d) for sensor %d, ignoring.\n", newRoi, i);
          break;
        }

        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        sensors[i].setDistanceMode((VL53L1X::DistanceMode)newMode);
        sensors[i].setROICenter(newRoi);
        xSemaphoreGive(sensorMutex);

        Serial.printf("[CAN RX] Sensor %d updated: mode=%d, roiCenter=%d\n", i, newMode, newRoi);
        break;
      }
    }
  }
}


void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize CAN on GPIO 4 (TX) and GPIO 5 (RX)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  delay(100);  // Ensure all sensors are off

  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(xshutPins[i], HIGH);
    delay(300);

    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.printf("Sensor %d failed to initialize!\n", i);
      while (1);
    }

    sensors[i].setAddress(sensorAddresses[i]);
    sensors[i].setROICenter(roiCenters[i]);
    sensors[i].setDistanceMode(rangingModes[i]);
    sensors[i].setMeasurementTimingBudget(timingBudgets[i]);
    sensors[i].startContinuous(50);
  }

  Serial.println("Sensor0\tSensor1\tSensor2\tSensor3");

  sensorMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(TaskSensorRead, "SensorRead", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskSensorPrint, "SensorPrint", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskCANTx, "CANTx", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskCANRx, "CANRx", 4096, NULL, 1, NULL, 0);

}

void loop() {
  // FreeRTOS handles everything
}
