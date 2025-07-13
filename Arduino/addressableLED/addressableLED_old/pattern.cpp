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



uint16_t breatheStep(const CRGB& color, uint8_t brightness, uint8_t speed, uint16_t frame) {
  float minSpeed = 0.05f;
  float maxSpeed = 5.0f;
  float scale = minSpeed + ((maxSpeed - minSpeed) * (speed / 255.0f));

  float t = frame * scale * 0.01f;
  float breath = 0.5f * (1 + sinf(t * 2 * PI));  // 0.0–1.0

  CRGB c = color;
  c.nscale8_video((uint8_t)(brightness * breath));
  fill_solid(leds, NUM_LEDS, c);
  FastLED.show();

  return frame + 1;
}

uint16_t fastBlinking(const CRGB& color, uint8_t speed, uint16_t frame) { 
  bool on = (frame / speed + 1) % 2 == 0;
  fill_solid(leds, NUM_LEDS, on ? color : CRGB::Black);
  FastLED.show();
  return frame + 1;
}

//uint16_t customPattern(const CRGB& color, uint8_t canParam0_IN,  uint8_t canParam1_IN, uint16_t frame) { //yourNewPatternStep
//  Your code goes here
//
//
//  FastLED.show(); //Update FastLED buffer
//}
