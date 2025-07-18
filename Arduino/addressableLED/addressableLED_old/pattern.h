#ifndef PATTERN_H
#define PATTERN_H

#include <Arduino.h>    // for delay(), uint8_t, uint16_t
#include <FastLED.h>    // for CRGB, fill_solid(), CHSV, FastLED.show()

extern const uint16_t NUM_LEDS;
extern       CRGB   leds[];

// Share brightness variable
extern volatile uint8_t canBrig;


// Animation prototypes
uint16_t colorWipeStep(const CRGB& color, uint8_t delayMs, uint16_t index);
uint16_t rainbowStep(uint8_t delayMs, uint16_t j);
uint16_t breatheStep(const CRGB& color, uint8_t maxBrightness, uint8_t speed, uint16_t frame);
uint16_t fastBlinking(const CRGB& color, uint8_t speed, uint16_t frame);
//uint16_t customPattern(const CRGB& color, uint8_t speed, uint16_t frame); //define yuor function


#endif // PATTERN_H
