#if !ARDUINO_USB_CDC_ON_BOOT
#error "ESP32-S3 USB CDC is required. Set Tools > USB CDC On Boot to Enabled."
#endif

#include <Wire.h>
#include <VL53L1X.h>
#include <FastLED.h>
#include <EEPROM.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/twai.h"

#define XSHUT_PIN 6
#define SDA_PIN   5
#define SCL_PIN   4
#define CAN_TX_PIN GPIO_NUM_9
#define CAN_RX_PIN GPIO_NUM_10
#define LED_PIN 21
#define LED_COUNT 1
#define LED_BRIGHTNESS 50
#define MAX_BLINK_DISTANCE_MM 50
#define NO_BLINK_DISTANCE_MM 1000
#define MAX_BLINK_FREQUENCY_HZ 50
#define LED_TASK_PERIOD_MS 5
#define NUM_SENSORS 1

#define DEVICE_TYPE_ID        0x0A
#define MANUFACTURER_ID  0x08
#define DEFAULT_DEVICE_NUMBER 50
#define EEPROM_SIZE_BYTES 8
#define EEPROM_ADDR_DEVICE_NUMBER 0
#define SENSOR_BASE_API_ID 0x0301
#define SENSOR_CONFIG_API_ID 0x0305

VL53L1X sensors[NUM_SENSORS];
int sensorDistances[NUM_SENSORS];
CRGB leds[LED_COUNT];
volatile uint8_t deviceNumber = DEFAULT_DEVICE_NUMBER;

const uint8_t xshutPins[NUM_SENSORS] = {XSHUT_PIN};
const uint8_t roiX[NUM_SENSORS] = {8};  // ROI width
const uint8_t roiY[NUM_SENSORS] = {8};  // ROI height
const uint8_t roiCenters[NUM_SENSORS] = {199};
const uint32_t timingBudgets[NUM_SENSORS] = {50000};
const VL53L1X::DistanceMode rangingModes[NUM_SENSORS] = {
  VL53L1X::Long
};

SemaphoreHandle_t sensorMutex;

uint32_t makeCANMsgID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID) << 24) |
         ((uint32_t)(manufacturerID) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) |
         (deviceNumber & 0x3F);
}

void loadDeviceNumber() {
  uint8_t storedDeviceNumber = EEPROM.read(EEPROM_ADDR_DEVICE_NUMBER);
  if (storedDeviceNumber <= 63) {
    deviceNumber = storedDeviceNumber;
  } else {
    deviceNumber = DEFAULT_DEVICE_NUMBER;
    EEPROM.write(EEPROM_ADDR_DEVICE_NUMBER, deviceNumber);
    EEPROM.commit();
  }
  Serial.printf("[CANID] Loaded device number: %u\n", deviceNumber);
}

void saveDeviceNumber() {
  EEPROM.write(EEPROM_ADDR_DEVICE_NUMBER, deviceNumber);
  EEPROM.commit();
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
    vTaskDelay(pdMS_TO_TICKS(3));  
  }
}

void TaskSensorPrint(void* pvParams) {
  for (;;) {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    for (int i = 0; i < NUM_SENSORS; i++) {
      int dist = sensorDistances[i];
      bool timeout = sensors[i].timeoutOccurred();
      uint8_t status = sensors[i].ranging_data.range_status;

      Serial.printf("Sensor %d: %d mm", i, dist);

      if (timeout || dist <= 0 || dist > 4000 || status != 0) {
        Serial.print(" [Error: ");
        if (timeout) Serial.print("Timeout ");
        if (dist <= 0 || dist > 4000) Serial.print("InvalidDist ");
        if (status != 0) Serial.printf("Status=%d ", status);
        Serial.print("]");
      }

      if (i < NUM_SENSORS - 1) Serial.print('\t');
    }
    Serial.println();
    xSemaphoreGive(sensorMutex);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void TaskLED(void* pvParams) {
  bool ledOn = false;
  bool displayedLedOn = false;
  uint32_t lastToggleMs = millis();

  for (;;) {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    int distance = sensorDistances[0];
    xSemaphoreGive(sensorMutex);

    if (distance == 0) {
      // A zero reading represents a timeout or invalid measurement.
      ledOn = true;
    } else if (distance >= NO_BLINK_DISTANCE_MM) {
      ledOn = false;
    } else {
      // Blink linearly from 50 Hz at 50 mm to 0 Hz at 1000 mm.
      uint16_t clampedDistance = max(distance, MAX_BLINK_DISTANCE_MM);
      uint32_t halfPeriodMs =
        ((NO_BLINK_DISTANCE_MM - MAX_BLINK_DISTANCE_MM) * 500UL) /
        (MAX_BLINK_FREQUENCY_HZ * (NO_BLINK_DISTANCE_MM - clampedDistance));

      if (millis() - lastToggleMs >= halfPeriodMs) {
        ledOn = !ledOn;
        lastToggleMs = millis();
      }
    }

    if (ledOn != displayedLedOn) {
      leds[0] = ledOn ? CRGB::Red : CRGB::Black;
      FastLED.show();
      displayedLedOn = ledOn;
    }

    vTaskDelay(pdMS_TO_TICKS(LED_TASK_PERIOD_MS));
  }
}


void TaskCANTx(void* pvParams) {
  for (;;) {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    for (int i = 0; i < NUM_SENSORS; i++) {
      int dist = sensorDistances[i];
      uint32_t tb = sensors[i].getMeasurementTimingBudget();

      uint8_t roiX = 0, roiY = 0;
      sensors[i].getROISize(&roiX, &roiY);

      twai_message_t msg = {};
      msg.identifier = makeCANMsgID(DEVICE_TYPE_ID, MANUFACTURER_ID, SENSOR_BASE_API_ID + i, deviceNumber);
      msg.extd = 1;
      msg.data_length_code = 8;

      msg.data[0] = (dist >> 8) & 0xFF;             // Distance high byte
      msg.data[1] = dist & 0xFF;                    // Distance low byte
      msg.data[2] = sensors[i].getDistanceMode();   // Ranging mode
      msg.data[3] = sensors[i].getROICenter();      // ROI center
      msg.data[4] = roiX;                           // ROI width
      msg.data[5] = roiY;                           // ROI height
      msg.data[6] = (tb >> 8) & 0xFF;               // Timing budget high byte
      msg.data[7] = tb & 0xFF;                      // Timing budget low byte

      twai_transmit(&msg, pdMS_TO_TICKS(1));
    }
    xSemaphoreGive(sensorMutex);
    vTaskDelay(pdMS_TO_TICKS(3));
  }
}


void TaskCANRx(void* pvParams) {
  for (;;) {
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(10)) != ESP_OK) continue;

    if (!msg.extd || msg.data_length_code < 2) continue;

    uint16_t apiId = (msg.identifier >> 6) & 0x3FF;
    uint8_t deviceNum = msg.identifier & 0x3F;
    if (deviceNum != deviceNumber) continue;

    for (int i = 0; i < NUM_SENSORS; i++) {
      if (apiId == SENSOR_CONFIG_API_ID + i) {
        uint8_t newMode   = msg.data[0];
        uint8_t newCenter = msg.data[1];
        uint8_t newRoiX   = (msg.data_length_code >= 4) ? msg.data[2] : 0;
        uint8_t newRoiY   = (msg.data_length_code >= 4) ? msg.data[3] : 0;

        if (newMode > 2) {
          Serial.printf("[CAN RX] Invalid mode (%d) for sensor %d, ignoring.\n", newMode, i);
          break;
        }

        if (newCenter > 255 || (newRoiX && (newRoiX < 4 || newRoiX > 16)) || (newRoiY && (newRoiY < 4 || newRoiY > 16))) {
          Serial.printf("[CAN RX] Invalid ROI values (center=%d, x=%d, y=%d) for sensor %d\n", newCenter, newRoiX, newRoiY, i);
          break;
        }

        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        sensors[i].setDistanceMode((VL53L1X::DistanceMode)newMode);
        
        if (newRoiX && newRoiY) {
          sensors[i].setROISize(newRoiX, newRoiY);
        }
        xSemaphoreGive(sensorMutex);
        sensors[i].setROICenter(newCenter);
        Serial.printf("[CAN RX] Sensor %d updated: mode=%d, center=%d, ROI=%dx%d\n", i, newMode, newCenter, newRoiX, newRoiY);
        break;
      }
    }
  }
}

void TaskCANIDHelper(void* pvParams) {
  Serial.println("[CANID] Commands: &CANID SET <0-63> / &CANID SAVE / &CANID GET");

  for (;;) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.startsWith("&CANID SET ")) {
        int newDeviceNumber = line.substring(11).toInt();
        if (newDeviceNumber >= 0 && newDeviceNumber <= 63) {
          deviceNumber = static_cast<uint8_t>(newDeviceNumber);
          Serial.printf("[CANID] Running device number set to %u\n", deviceNumber);
        } else {
          Serial.println("[CANID] Invalid value. Must be 0-63.");
        }
      } else if (line.equals("&CANID SAVE")) {
        saveDeviceNumber();
        Serial.println("[CANID] Saved to EEPROM. Rebooting...");
        Serial.flush();
        delay(1000);
        ESP.restart();
      } else if (line.equals("&CANID GET")) {
        uint8_t savedDeviceNumber = EEPROM.read(EEPROM_ADDR_DEVICE_NUMBER);
        Serial.printf("[CANID] Current=%u, EEPROM=%u, Default=%u\n",
                      deviceNumber, savedDeviceNumber, DEFAULT_DEVICE_NUMBER);
      } else if (line.length() > 0) {
        Serial.println("[CANID] Unknown command. Use &CANID SET <0-63> / SAVE / GET");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}


void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE_BYTES);
  loadDeviceNumber();
  Wire.begin(SDA_PIN, SCL_PIN);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear(true);

  // CAN/TWAI: GP9 TX, GP10 RX.
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
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

    sensors[i].setROISize(roiX[i], roiY[i]); 
    sensors[i].setROICenter(roiCenters[i]);
    sensors[i].setDistanceMode(rangingModes[i]);
    sensors[i].setMeasurementTimingBudget(timingBudgets[i]);
    delay(5);

    sensors[i].startContinuous(50);
  }

  Serial.println("Sensor0");

  sensorMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(TaskSensorRead, "SensorRead", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskSensorPrint, "SensorPrint", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskLED, "LED", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskCANTx, "CANTx", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskCANRx, "CANRx", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskCANIDHelper, "CANID", 4096, NULL, 1, NULL, 0);

}

void loop() {
  // FreeRTOS handles everything
}
