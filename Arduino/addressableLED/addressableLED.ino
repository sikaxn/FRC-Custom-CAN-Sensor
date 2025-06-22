#include <Arduino.h>
#include <FastLED.h>
#include "pattern.h"

// —— Configuration ——
const uint16_t NUM_LEDS  = 60;
CRGB          leds[NUM_LEDS];

#define DATA_PIN    16       // Avoids 4,5
#define BRIGHTNESS  128
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

// —— Animation timing —— 
const uint32_t interval    = 10000;  // switch every 10 s
uint32_t       lastSwap    = 0;
uint8_t        currentAnim = 0;

// —— LED task —— 
void TaskLEDWrite(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    uint32_t now = millis();
    if (now - lastSwap > interval) {
      lastSwap   = now;
      currentAnim = (currentAnim + 1) % 2;
    }

    if (currentAnim == 0) {
      // color wipe sequence
      colorWipe(CRGB::Red,   20);
      colorWipe(CRGB::Green, 20);
      colorWipe(CRGB::Blue,  20);
    } else {
      // smooth rainbow
      rainbowCycle(10);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting NeoPixel demo on GPIO16…");

  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
         .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  // Start LED loop on core 1 (pin your other tasks to core 0 if desired)
  xTaskCreatePinnedToCore(
    TaskLEDWrite,    // function
    "LED Task",      // name (for debugging)
    4096,            // stack size
    NULL,            // parameter
    1,               // priority
    NULL,            // task handle
    1                // run on core 1
  );
}

void loop() {
  // All the work happens in TaskLEDWrite.
  // Just yield here to keep the idle task happy:
  vTaskDelay(portMAX_DELAY);
}
