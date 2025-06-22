#include "pattern.h"


// Lights each LED in turn, then clears
void colorWipe(const CRGB& color, uint16_t wait) {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
    FastLED.show();
    delay(wait);
  }
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

// Runs a continuously shifting rainbow
void rainbowCycle(uint8_t wait) {
  static uint16_t j = 0;
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV((i * 256 / NUM_LEDS + j) & 255, 255, 255);
  }
  FastLED.show();
  delay(wait);
  j++;
}