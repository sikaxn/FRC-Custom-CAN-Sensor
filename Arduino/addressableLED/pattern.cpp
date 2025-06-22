#include "pattern.h"

uint16_t colorWipeStep(const CRGB& color, uint8_t delayMs, uint16_t index) {
  if (index < NUM_LEDS) {
    leds[index] = color;
    FastLED.show();
    delay(delayMs);
    return index + 1;
  } else {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    return 0;
  }
}

uint16_t rainbowStep(uint8_t delayMs, uint16_t j) {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV((i * 256 / NUM_LEDS + j) & 255, 255, canBrig);
  }
  FastLED.show();
  delay(delayMs);
  return j + 1;
}
