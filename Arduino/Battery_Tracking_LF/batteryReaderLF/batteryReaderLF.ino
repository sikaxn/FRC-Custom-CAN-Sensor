#include <Arduino.h>
#include <HardwareSerial.h>

// =============================
// --- Pin configuration ---
// =============================
#define RFID_RX_PIN 26   // Reader TX → ESP32 RX
#define RFID_TX_PIN 34   // Optional (unused for read-only)

// =============================
// --- Hardware Serial instance ---
// =============================
HardwareSerial rfidSerial(1);

// =============================
// --- Global shared variables ---
// =============================
String lfTagSerial = "";       // Holds current tag serial
bool lfReaderActive = false;   // true = tag present
SemaphoreHandle_t lfMutex;     // Protects access to above variables

// =============================
// --- FreeRTOS Task: LF Reader ---
// =============================
void TaskHandleLFReader(void* pvParameters) {
  rfidSerial.begin(9600, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
  Serial.println(F("[LFReader] Task started. Waiting for tag..."));

  static String buffer = "";
  static String lastTag = "";
  unsigned long lastReadTime = 0;
  bool tagPresent = false;

  for (;;) {
    // --- Read and parse incoming UART data ---
    while (rfidSerial.available()) {
      char c = rfidSerial.read();

      // Accept only printable chars, STX (0x02), ETX (0x03)
      if ((c >= 32 && c <= 126) || c == 0x02 || c == 0x03) {
        buffer += c;

        // Look for complete frame between STX (0x02) and ETX (0x03)
        int startIdx = buffer.indexOf(0x02);
        int endIdx   = buffer.indexOf(0x03, startIdx + 1);

        if (startIdx >= 0 && endIdx > startIdx) {
          String frame = buffer.substring(startIdx + 1, endIdx);
          buffer = "";  // clear buffer after successful read

          // Validate reasonable tag length
          if (frame.length() >= 10 && frame.length() <= 16) {
            if (frame != lastTag) {
              lastTag = frame;
              tagPresent = true;
              Serial.print(F("[LFReader] New tag detected: "));
              Serial.println(lastTag);
            }
            lastReadTime = millis();
          }
        }

        // Prevent runaway buffer
        if (buffer.length() > 32)
          buffer.remove(0, buffer.length() - 16);
      }
    }

    // --- Detect tag removal (no data > 1000 ms) ---
    if (tagPresent && millis() - lastReadTime > 1000) {
      Serial.print(F("[LFReader] Tag removed: "));
      Serial.println(lastTag);
      tagPresent = false;
      lastTag = "";
    }

    // --- Update global variables safely ---
    if (lfMutex && xSemaphoreTake(lfMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      lfReaderActive = tagPresent;
      lfTagSerial = tagPresent ? lastTag : "";
      xSemaphoreGive(lfMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(20));  // yield to other tasks
  }
}

// =============================
// --- Example: Reader Monitor Task ---
// =============================
void TaskMonitorLF(void* pvParameters) {
  for (;;) {
    if (lfMutex && xSemaphoreTake(lfMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      bool active = lfReaderActive;
      String tag = lfTagSerial;
      xSemaphoreGive(lfMutex);

      if (active)
        Serial.printf("[Monitor] Tag present: %s\n", tag.c_str());
      else
        Serial.println("[Monitor] No tag detected");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// =============================
// --- Setup ---
// =============================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=== RDM6300 LF Reader Example ==="));

  lfMutex = xSemaphoreCreateMutex();

  // Create tasks
  xTaskCreatePinnedToCore(TaskHandleLFReader, "LFReader", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskMonitorLF, "MonitorLF", 2048, NULL, 1, NULL, 1);

  Serial.println(F("[System] Tasks started. Place tag near RDM6300 reader."));
}

// =============================
// --- Loop (unused) ---
// =============================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
