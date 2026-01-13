#include "mode.h"
#include "pattern.h"

extern volatile uint8_t canMode, canR, canG, canB, canBrig, canParam0, canParam1;
extern volatile bool canOnOff;

extern volatile bool modeRefresh;
extern CRGB leds[];
extern uint16_t NUM_LEDS;

static uint16_t modeFrame = 0;
static int16_t bouncePos = 0;
static int8_t bounceDir = 1;
static int16_t centerBounceOffset = 0;
static int8_t centerBounceDir = 1;
static int16_t centerWipeOffset = 0;

void runCurrentMode() {
  switch (canMode) {
    case 0:  // Off
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      break;

    case 1:  // Solid color
      if (canOnOff) {
        CRGB color = CRGB{canR, canG, canB};
        color.nscale8_video(canBrig);
        fill_solid(leds, NUM_LEDS, color);
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
      }
      FastLED.show();
      break;

    case 2:
      modeFrame = colorWipeStep(CRGB{canR, canG, canB}, canParam0, modeFrame);
      break;


    case 3:
      if (modeRefresh) {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
        modeFrame = 0;
        modeRefresh = false;
      }
      modeFrame = colorWipeStep(CRGB{canR, canG, canB}, canParam0, modeFrame);
      break;


    case 4:
      modeFrame = rainbowStep(canParam0, modeFrame);
      break;

      
    case 5: {  // Breathe with reset
      if (modeRefresh) {
        modeFrame = 0;
        modeRefresh = false;
      }
      modeFrame = breatheStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame);
      break;
    }

    case 6: {  // Breathe no reset
      modeFrame = breatheStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame);
      break;
    }

    case 7: { //sample yourNewPatternStep
      if (modeRefresh) { //if you want a complete reset when switch to this mode or received a new setting, add these.
        modeFrame = 0;
        modeRefresh = false;
      }
      modeFrame = fastBlinking(CRGB{canR, canG, canB}, canParam0, modeFrame);
      break;
    }

    case 8: { // single pixel wipe
      if (modeRefresh) {
        modeFrame = 0;
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      modeFrame = singlePixelWipeStep(CRGB{canR, canG, canB}, canParam0, modeFrame, length);
      break;
    }

      case 9: { // single pixel wipe no reset
      if (modeRefresh) {
        //modeFrame = 0;
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      modeFrame = singlePixelWipeStep(CRGB{canR, canG, canB}, canParam0, modeFrame, length);
      break;
    }
    

    case 10: { // single pixel bounce
      if (modeRefresh) {
        bouncePos = 0;
        bounceDir = 1;
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      bouncePos = (int16_t)singlePixelBounceStep(CRGB{canR, canG, canB}, canParam0, bouncePos, bounceDir, length);
      break;
    }

    case 11: { // single pixel bounce no reset
      if (modeRefresh) {
        //bouncePos = 0;
        //bounceDir = 1;
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      bouncePos = (int16_t)singlePixelBounceStep(CRGB{canR, canG, canB}, canParam0, bouncePos, bounceDir, length);
      break;
    }

    case 12: { // center wipe
      if (modeRefresh) {
        centerWipeOffset = 0;
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      centerWipeOffset = (int16_t)centerWipeStep(CRGB{canR, canG, canB}, canParam0, centerWipeOffset, length);
      break;
    }

    case 13: { // center wipe no reset
      if (modeRefresh) {
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      centerWipeOffset = (int16_t)centerWipeStep(CRGB{canR, canG, canB}, canParam0, centerWipeOffset, length);
      break;
    }

    case 14: { // center bounce
      if (modeRefresh) {
        centerBounceOffset = 0;
        centerBounceDir = 1;
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      centerBounceOffset = (int16_t)centerBounceStep(CRGB{canR, canG, canB}, canParam0, centerBounceOffset, centerBounceDir, length);
      break;
    }

    case 15: { // center bounce no reset
      if (modeRefresh) {
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      centerBounceOffset = (int16_t)centerBounceStep(CRGB{canR, canG, canB}, canParam0, centerBounceOffset, centerBounceDir, length);
      break;
    }

    case 16: { // alternating blocks no reset
      if (modeRefresh) {
        modeRefresh = false;
      }
      uint8_t delayMs = (uint8_t)(255 - canParam0);
      if (delayMs == 0) {
        delayMs = 1;
      }
      modeFrame = alternatingBlockStep(CRGB{canR, canG, canB}, delayMs, modeFrame, canParam1);
      break;
    }

    case 17: { // alternating block fade no reset
      if (modeRefresh) {
        modeRefresh = false;
      }
      modeFrame = alternatingBlockFadeStep(CRGB{canR, canG, canB}, canParam0, modeFrame, canParam1);
      break;
    }

    case 18: { // alternating block fade no reset
      if (modeRefresh) {
        modeRefresh = false;
      }
      modeFrame = alternatingBlockFadeStep(CRGB{canR, canG, canB}, canParam0, modeFrame, canParam1);
      break;
    }

    //ADD Your Own Mode Here
    //case N:{
      //if (modeRefresh) { //if you want a complete reset when switch to this mode or received a new setting, add these.
      //  modeFrame = 0;
      //  modeRefresh = false;
      //}
    //  Your code goes here
    //}
    //


    case 255: { // custom pixel mode
      if (modeRefresh) {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
        modeRefresh = false;
      }

      if (customSeen && customPix < NUM_LEDS) {
        leds[customPix] = CRGB{cR, cG, cB};
        FastLED.show();
        customSeen = false;
      }
      break;
    }


    default: //invalid case
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      break;
  }
}
