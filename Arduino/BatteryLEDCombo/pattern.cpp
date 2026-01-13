#include "pattern.h"

uint16_t colorWipeStep(const CRGB& color, uint8_t delayMs, uint16_t index) {
  if (index < NUM_LEDS) {
    CRGB c = color;
    c.nscale8_video(canBrig);
    leds[index] = c;
    FastLED.show();
    delay(delayMs);
    return index + 1;
  } else {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    return 0;
  }
}

uint16_t singlePixelWipeStep(const CRGB& color, uint8_t delayMs, uint16_t index, uint16_t length) {
  if (length == 0 || NUM_LEDS == 0) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    delay(delayMs);
    return 0;
  }

  if (length > NUM_LEDS) {
    length = NUM_LEDS;
  }

  uint16_t pos = index % NUM_LEDS;
  CRGB c = color;
  c.nscale8_video(canBrig);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  for (uint16_t i = 0; i < length; i++) {
    leds[(pos + i) % NUM_LEDS] = c;
  }
  FastLED.show();
  delay(delayMs);
  return pos + 1;
}

uint16_t singlePixelBounceStep(const CRGB& color, uint8_t delayMs, int16_t pos, int8_t& dir, uint16_t length) {
  if (length == 0 || NUM_LEDS == 0) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    delay(delayMs);
    return 0;
  }

  if (length > NUM_LEDS) {
    length = NUM_LEDS;
  }

  int16_t maxPos = (int16_t)NUM_LEDS - (int16_t)length;
  if (maxPos < 0) {
    maxPos = 0;
  }

  if (pos < 0) {
    pos = 0;
    dir = 1;
  } else if (pos > maxPos) {
    pos = maxPos;
    dir = -1;
  }

  CRGB c = color;
  c.nscale8_video(canBrig);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  for (uint16_t i = 0; i < length; i++) {
    leds[pos + i] = c;
  }
  FastLED.show();
  delay(delayMs);

  if (maxPos == 0) {
    return 0;
  }

  if (pos == 0 && dir < 0) {
    dir = 1;
  } else if (pos == maxPos && dir > 0) {
    dir = -1;
  }

  return pos + dir;
}

uint16_t centerBounceStep(const CRGB& color, uint8_t delayMs, int16_t offset, int8_t& dir, uint16_t length) {
  if (length == 0 || NUM_LEDS == 0) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    delay(delayMs);
    return 0;
  }

  if (length > NUM_LEDS) {
    length = NUM_LEDS;
  }

  int16_t centerLeft = (int16_t)((NUM_LEDS - 1) / 2);
  int16_t centerRight = (int16_t)(NUM_LEDS / 2);

  int16_t maxOffsetLeft = centerLeft - (int16_t)(length - 1);
  int16_t maxOffsetRight = (int16_t)(NUM_LEDS - 1) - centerRight - (int16_t)(length - 1);
  int16_t maxOffset = maxOffsetLeft < maxOffsetRight ? maxOffsetLeft : maxOffsetRight;
  if (maxOffset < 0) {
    maxOffset = 0;
  }

  if (offset < 0) {
    offset = 0;
    dir = 1;
  } else if (offset > maxOffset) {
    offset = maxOffset;
    dir = -1;
  }

  CRGB c = color;
  c.nscale8_video(canBrig);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  for (uint16_t i = 0; i < length; i++) {
    int16_t leftIndex = centerLeft - offset - (int16_t)i;
    int16_t rightIndex = centerRight + offset + (int16_t)i;
    if (leftIndex >= 0 && leftIndex < (int16_t)NUM_LEDS) {
      leds[leftIndex] = c;
    }
    if (rightIndex >= 0 && rightIndex < (int16_t)NUM_LEDS) {
      leds[rightIndex] = c;
    }
  }
  FastLED.show();
  delay(delayMs);

  if (maxOffset == 0) {
    return 0;
  }

  if (offset == 0 && dir < 0) {
    dir = 1;
  } else if (offset == maxOffset && dir > 0) {
    dir = -1;
  }

  return offset + dir;
}

uint16_t centerWipeStep(const CRGB& color, uint8_t delayMs, int16_t offset, uint16_t length) {
  if (length == 0 || NUM_LEDS == 0) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    delay(delayMs);
    return 0;
  }

  if (length > NUM_LEDS) {
    length = NUM_LEDS;
  }

  int16_t centerLeft = (int16_t)((NUM_LEDS - 1) / 2);
  int16_t centerRight = (int16_t)(NUM_LEDS / 2);

  int16_t maxOffsetLeft = centerLeft - (int16_t)(length - 1);
  int16_t maxOffsetRight = (int16_t)(NUM_LEDS - 1) - centerRight - (int16_t)(length - 1);
  int16_t maxOffset = maxOffsetLeft < maxOffsetRight ? maxOffsetLeft : maxOffsetRight;
  if (maxOffset < 0) {
    maxOffset = 0;
  }

  if (offset < 0) {
    offset = 0;
  } else if (offset > maxOffset) {
    offset = 0;
  }

  CRGB c = color;
  c.nscale8_video(canBrig);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  for (uint16_t i = 0; i < length; i++) {
    int16_t leftIndex = centerLeft - offset - (int16_t)i;
    int16_t rightIndex = centerRight + offset + (int16_t)i;
    if (leftIndex >= 0 && leftIndex < (int16_t)NUM_LEDS) {
      leds[leftIndex] = c;
    }
    if (rightIndex >= 0 && rightIndex < (int16_t)NUM_LEDS) {
      leds[rightIndex] = c;
    }
  }
  FastLED.show();
  delay(delayMs);

  if (maxOffset == 0) {
    return 0;
  }

  return offset + 1;
}

uint16_t alternatingBlockStep(const CRGB& color, uint8_t delayMs, uint16_t frame, uint8_t spacing) {
  uint16_t blockSize = spacing == 0 ? 1 : spacing;
  uint16_t period = blockSize * 2;

  CRGB c = color;
  c.nscale8_video(canBrig);

  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    uint16_t phase = (i + (uint16_t)frame * blockSize) % period;
    leds[i] = (phase < blockSize) ? c : CRGB::Black;
  }
  FastLED.show();
  delay(delayMs);
  return frame + 1;
}

uint16_t alternatingBlockFadeStep(const CRGB& color, uint8_t delayMs, uint16_t frame, uint8_t spacing) {
  uint16_t blockSize = spacing == 0 ? 1 : spacing;
  uint16_t period = blockSize * 2;

  uint8_t fade = frame & 0xFF;
  bool useAlt = ((frame >> 8) & 0x01) != 0;

  CRGB c = color;
  c.nscale8_video(canBrig);

  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    uint16_t phaseA = i % period;
    uint16_t phaseB = (i + blockSize) % period;
    bool onA = (phaseA < blockSize);
    bool onB = (phaseB < blockSize);

    uint16_t aWeight = useAlt ? (uint16_t)fade : (uint16_t)(255 - fade);
    uint16_t bWeight = 255 - aWeight;

    uint16_t mix = (onA ? aWeight : 0) + (onB ? bWeight : 0);
    CRGB out = c;
    out.nscale8_video((uint8_t)mix);
    leds[i] = out;
  }
  FastLED.show();
  delay(delayMs);
  return frame + 1;
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
  CRGB c = color;
  c.nscale8_video(canBrig);
  fill_solid(leds, NUM_LEDS, on ? c : CRGB::Black);
  FastLED.show();
  return frame + 1;
}

//uint16_t customPattern(const CRGB& color, uint8_t canParam0_IN,  uint8_t canParam1_IN, uint16_t frame) { //yourNewPatternStep
//  Your code goes here
//
//
//  FastLED.show(); //Update FastLED buffer
//}
