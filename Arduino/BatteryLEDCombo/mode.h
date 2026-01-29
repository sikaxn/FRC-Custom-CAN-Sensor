#pragma once
#include <FastLED.h>

// Called every loop of TaskLEDWrite
void runCurrentMode();

// Expose shared state for all mode logic
// Use a pointer on host builds for dynamic sizing.
#ifdef LEDSIM_HOST
extern CRGB* leds;
#else
extern CRGB leds[];
#endif
extern uint16_t NUM_LEDS;
extern volatile uint8_t canMode;
extern volatile uint8_t canR, canG, canB, canBrig;
extern volatile uint8_t canR2, canG2, canB2, canBrig2, canOnOff2;
extern volatile bool canOnOff;
extern volatile uint8_t canParam0, canParam1;
extern volatile bool customSeen;
extern volatile uint16_t customPix;
extern volatile uint8_t cR, cG, cB;
