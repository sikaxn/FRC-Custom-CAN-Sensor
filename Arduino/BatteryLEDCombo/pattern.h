#ifndef PATTERN_H
#define PATTERN_H

#include <Arduino.h>    // for delay(), uint8_t, uint16_t
#include <FastLED.h>    // for CRGB, fill_solid(), CHSV, FastLED.show()

extern uint16_t NUM_LEDS;
// Use a pointer on host builds for dynamic sizing.
#ifdef LEDSIM_HOST
extern CRGB* leds;
#else
extern       CRGB   leds[];
#endif

// Animation prototypes
uint16_t colorWipeStep(const CRGB& color, uint8_t brightness, uint8_t delayMs, uint16_t index,
                       const CRGB& background = CRGB::Black, uint8_t backgroundBrightness = 0);
uint16_t singlePixelWipeStep(const CRGB& color, uint8_t brightness, uint8_t delayMs, uint16_t index, uint16_t length,
                             const CRGB& background = CRGB::Black, uint8_t backgroundBrightness = 0);
uint16_t singlePixelBounceStep(const CRGB& color, uint8_t brightness, uint8_t delayMs, int16_t pos, int8_t& dir, uint16_t length,
                               const CRGB& background = CRGB::Black, uint8_t backgroundBrightness = 0);
uint16_t centerBounceStep(const CRGB& color, uint8_t brightness, uint8_t delayMs, int16_t offset, int8_t& dir, uint16_t length,
                          const CRGB& background = CRGB::Black, uint8_t backgroundBrightness = 0);
uint16_t centerWipeStep(const CRGB& color, uint8_t brightness, uint8_t delayMs, int16_t offset, uint16_t length,
                        const CRGB& background = CRGB::Black, uint8_t backgroundBrightness = 0);
uint16_t alternatingBlockStep(const CRGB& color, uint8_t brightness, uint8_t delayMs, uint16_t frame, uint8_t spacing,
                              const CRGB& background = CRGB::Black, uint8_t backgroundBrightness = 0);
uint16_t alternatingBlockFadeStep(const CRGB& color, uint8_t brightness, uint8_t delayMs, uint16_t frame, uint8_t spacing,
                                  const CRGB& background = CRGB::Black, uint8_t backgroundBrightness = 0);
uint16_t rainbowStep(uint8_t brightness, uint8_t delayMs, uint16_t j);
void powerOnSequenceStep(uint8_t& phase, uint32_t& phaseStartMs);
uint16_t breatheStep(const CRGB& color, uint8_t maxBrightness, uint8_t speed, uint16_t frame,
                     const CRGB& background = CRGB::Black, uint8_t backgroundBrightness = 0);
uint16_t fastBlinking(const CRGB& color, uint8_t brightness, uint8_t speed, uint16_t frame,
                      const CRGB& background = CRGB::Black, uint8_t backgroundBrightness = 0);
//uint16_t customPattern(const CRGB& color, uint8_t speed, uint16_t frame); //define yuor function


#endif // PATTERN_H
