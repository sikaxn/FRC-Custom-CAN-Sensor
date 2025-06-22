#ifndef PATTERN_H
#define PATTERN_H

#include <Arduino.h>    // for delay(), uint8_t, uint16_t
#include <FastLED.h>    // for CRGB, fill_solid(), CHSV, FastLED.show()

// These live in .ino:
extern const uint16_t NUM_LEDS;
extern       CRGB   leds[];

// Animation prototypes
void colorWipe(const CRGB& color, uint16_t wait);
void rainbowCycle(uint8_t wait);

#endif // PATTERN_H
