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


void breathePattern(const CRGB& color, uint8_t speed, uint32_t& frame) {
  // Compute brightness multiplier (0–255) using sine wave
  float angle = (frame % 256) * PI / 128.0;  // full cycle every 256 frames
  float brightness = (sin(angle) + 1.0) * 127.5;  // 0–255

  CRGB scaled = color;
  scaled.nscale8_video((uint8_t)brightness);

  fill_solid(leds, NUM_LEDS, scaled);
  FastLED.show();

  frame += speed;  // speed controls how fast it breathes
}