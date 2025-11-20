#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[DS3231 RTC Example]");

  // Initialize I2C with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!rtc.begin(&Wire)) {
    Serial.println("RTC NOT found!");
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power! Setting default time 2025-01-01 00:00:00");
    rtc.adjust(DateTime(2025, 1, 1, 0, 0, 0));
  }

  Serial.println("RTC initialized.");
  Serial.println("To set UTC time, use:");
  Serial.println("  &SETTIME yyyy-mm-dd hh:mm:ss\n");
}

void loop() {
  // PRINT TIME EVERY 5 SECONDS
  if (millis() - lastPrint >= 5000) {
    lastPrint = millis();
    DateTime now = rtc.now();
    Serial.printf("RTC: %04d-%02d-%02d %02d:%02d:%02d\n",
      now.year(), now.month(), now.day(),
      now.hour(), now.minute(), now.second());
  }

  // CHECK FOR SERIAL TIME SET COMMAND
  if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.startsWith("&SETTIME ")) {

        // Extract string after "&SETTIME "
        String datetime = line.substring(9);  // 9 characters: & S E T T I M E and space

        int y, mo, d, h, mi, s;

        if (sscanf(datetime.c_str(),
                  "%d-%d-%d %d:%d:%d",
                  &y, &mo, &d, &h, &mi, &s) == 6) {

          rtc.adjust(DateTime(y, mo, d, h, mi, s));
          Serial.println("[RTC] Time updated!");
        }
        else {
          Serial.println("[RTC] Invalid format. Use:");
          Serial.println("&SETTIME yyyy-mm-dd hh:mm:ss");
        }
      }
  }
}
