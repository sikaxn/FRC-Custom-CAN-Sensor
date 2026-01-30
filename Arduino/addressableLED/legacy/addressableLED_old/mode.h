#pragma once
#include <FastLED.h>

// Called every loop of TaskLEDWrite
void runCurrentMode();

// Expose shared state for all mode logic
extern CRGB leds[];
extern const uint16_t NUM_LEDS;
extern volatile uint8_t canMode;
extern volatile uint8_t canR, canG, canB, canBrig;
extern volatile bool canOnOff;
extern volatile uint8_t canParam0, canParam1;
extern volatile bool customSeen;
extern volatile uint16_t customPix;
extern volatile uint8_t cR, cG, cB;
