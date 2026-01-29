#include "mode.h"
#include "pattern.h"

extern volatile uint8_t canMode, canR, canG, canB, canBrig, canParam0, canParam1;
extern volatile uint8_t canR2, canG2, canB2, canBrig2, canOnOff2;
extern volatile bool canOnOff;

extern volatile bool modeRefresh;
#ifdef LEDSIM_HOST
extern CRGB* leds;
#else
extern CRGB leds[];
#endif
extern uint16_t NUM_LEDS;

static uint16_t modeFrame = 0;
static int16_t bouncePos = 0;
static int8_t bounceDir = 1;
static int16_t centerBounceOffset = 0;
static int8_t centerBounceDir = 1;
static int16_t centerWipeOffset = 0;
static uint8_t powerPhase = 0;
static uint32_t powerPhaseStartMs = 0;
static uint8_t lastMode = 0xFF;
static bool color2Pending = false;

static void getColor2(CRGB& colorOut, uint8_t& brigOut);

static bool isColor2Mode(uint8_t mode) {
  return mode >= 19 && mode <= 34;
}

static void applyColor2ToOffPixels() {
  CRGB color2;
  uint8_t brig2 = 0;
  getColor2(color2, brig2);
  if (brig2 == 0) {
    color2 = CRGB::Black;
  } else {
    color2.nscale8_video(brig2);
  }
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    if (leds[i] == CRGB::Black) {
      leds[i] = color2;
    }
  }
  FastLED.show();
}

static void getColor2(CRGB& colorOut, uint8_t& brigOut) {
  if (canOnOff2) {
    colorOut = CRGB{canR2, canG2, canB2};
    brigOut = (canBrig2 <= canBrig) ? canBrig2 : canBrig;
  } else {
    colorOut = CRGB::Black;
    brigOut = 0;
  }
}

void runCurrentMode() {
  if (canMode != lastMode) {
    color2Pending = (lastMode == 3 && isColor2Mode(canMode));
    lastMode = canMode;
  }

  switch (canMode) {
    case 0:  // Off
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      break;

    case 1:  // Solid color
      if (true) {
        CRGB color = CRGB{canR, canG, canB};
        color.nscale8_video(canBrig);
        fill_solid(leds, NUM_LEDS, color);
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
      }
      FastLED.show();
      break;

    case 2:
      modeFrame = colorWipeStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame);
      break;


    case 3:
      if (modeRefresh) {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
        modeFrame = 0;
        modeRefresh = false;
      }
      modeFrame = colorWipeStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame);
      break;


    case 4:
      modeFrame = rainbowStep(canBrig, canParam0, modeFrame);
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
      uint8_t speed = canParam0 == 0 ? 1 : canParam0;
      modeFrame = fastBlinking(CRGB{canR, canG, canB}, canBrig, speed, modeFrame);
      break;
    }

    case 8: { // single pixel wipe
      if (modeRefresh) {
        modeFrame = 0;
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      modeFrame = singlePixelWipeStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, length);
      break;
    }

      case 9: { // single pixel wipe no reset
      if (modeRefresh) {
        //modeFrame = 0;
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      modeFrame = singlePixelWipeStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, length);
      break;
    }
    

    case 10: { // single pixel bounce
      if (modeRefresh) {
        bouncePos = 0;
        bounceDir = 1;
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      bouncePos = (int16_t)singlePixelBounceStep(CRGB{canR, canG, canB}, canBrig, canParam0, bouncePos, bounceDir, length);
      break;
    }

    case 11: { // single pixel bounce no reset
      if (modeRefresh) {
        //bouncePos = 0;
        //bounceDir = 1;
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      bouncePos = (int16_t)singlePixelBounceStep(CRGB{canR, canG, canB}, canBrig, canParam0, bouncePos, bounceDir, length);
      break;
    }

    case 12: { // center wipe
      if (modeRefresh) {
        centerWipeOffset = 0;
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      centerWipeOffset = (int16_t)centerWipeStep(CRGB{canR, canG, canB}, canBrig, canParam0, centerWipeOffset, length);
      break;
    }

    case 13: { // center wipe no reset
      if (modeRefresh) {
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      centerWipeOffset = (int16_t)centerWipeStep(CRGB{canR, canG, canB}, canBrig, canParam0, centerWipeOffset, length);
      break;
    }

    case 14: { // center bounce
      if (modeRefresh) {
        centerBounceOffset = 0;
        centerBounceDir = 1;
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      centerBounceOffset = (int16_t)centerBounceStep(CRGB{canR, canG, canB}, canBrig, canParam0, centerBounceOffset, centerBounceDir, length);
      break;
    }

    case 15: { // center bounce no reset
      if (modeRefresh) {
        modeRefresh = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      centerBounceOffset = (int16_t)centerBounceStep(CRGB{canR, canG, canB}, canBrig, canParam0, centerBounceOffset, centerBounceDir, length);
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
      modeFrame = alternatingBlockStep(CRGB{canR, canG, canB}, canBrig, delayMs, modeFrame, canParam1);
      break;
    }

    case 17: { // alternating block fade reset
      if (modeRefresh) {
        modeFrame = 0;
        modeRefresh = false;
      }
      modeFrame = alternatingBlockFadeStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, canParam1);
      break;
    }

    case 18: { // alternating block fade no reset
      if (modeRefresh) {
        modeRefresh = false;
      }
      modeFrame = alternatingBlockFadeStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, canParam1);
      break;
    }

    case 19: { // color wipe with color 2 as background
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      modeFrame = colorWipeStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, color2, brig2);
      break;
    }

    case 20: { // color wipe reset with color 2 as background
      if (modeRefresh) {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
        modeFrame = 0;
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      modeFrame = colorWipeStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, color2, brig2);
      break;
    }

    case 21: { // breathe reset with color 2 as background
      if (modeRefresh) {
        modeFrame = 0;
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      modeFrame = breatheStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, color2, brig2);
      break;
    }

    case 22: { // breathe no reset with color 2 as background
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      modeFrame = breatheStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, color2, brig2);
      break;
    }

    case 23: { // fast blinking reset with color 2 as background
      if (modeRefresh) {
        modeFrame = 0;
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      modeFrame = fastBlinking(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, color2, brig2);
      break;
    }

    case 24: { // single pixel wipe reset with color 2 as background
      if (modeRefresh) {
        modeFrame = 0;
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      modeFrame = singlePixelWipeStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, length, color2, brig2);
      break;
    }

    case 25: { // single pixel wipe no reset with color 2 as background
      if (modeRefresh) {
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      modeFrame = singlePixelWipeStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, length, color2, brig2);
      break;
    }

    case 26: { // single pixel bounce reset with color 2 as background
      if (modeRefresh) {
        bouncePos = 0;
        bounceDir = 1;
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      bouncePos = (int16_t)singlePixelBounceStep(CRGB{canR, canG, canB}, canBrig, canParam0, bouncePos, bounceDir, length, color2, brig2);
      break;
    }

    case 27: { // single pixel bounce no reset with color 2 as background
      if (modeRefresh) {
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      bouncePos = (int16_t)singlePixelBounceStep(CRGB{canR, canG, canB}, canBrig, canParam0, bouncePos, bounceDir, length, color2, brig2);
      break;
    }

    case 28: { // center wipe reset with color 2 as background
      if (modeRefresh) {
        centerWipeOffset = 0;
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      centerWipeOffset = (int16_t)centerWipeStep(CRGB{canR, canG, canB}, canBrig, canParam0, centerWipeOffset, length, color2, brig2);
      break;
    }

    case 29: { // center wipe no reset with color 2 as background
      if (modeRefresh) {
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      centerWipeOffset = (int16_t)centerWipeStep(CRGB{canR, canG, canB}, canBrig, canParam0, centerWipeOffset, length, color2, brig2);
      break;
    }

    case 30: { // center bounce reset with color 2 as background
      if (modeRefresh) {
        centerBounceOffset = 0;
        centerBounceDir = 1;
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      centerBounceOffset = (int16_t)centerBounceStep(CRGB{canR, canG, canB}, canBrig, canParam0, centerBounceOffset, centerBounceDir, length, color2, brig2);
      break;
    }

    case 31: { // center bounce no reset with color 2 as background
      if (modeRefresh) {
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      uint16_t length = (uint16_t)canParam1 + 1;
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      centerBounceOffset = (int16_t)centerBounceStep(CRGB{canR, canG, canB}, canBrig, canParam0, centerBounceOffset, centerBounceDir, length, color2, brig2);
      break;
    }

    case 32: { // alternating blocks no reset with color 2 as background
      if (modeRefresh) {
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      uint8_t delayMs = (uint8_t)(255 - canParam0);
      if (delayMs == 0) {
        delayMs = 1;
      }
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      modeFrame = alternatingBlockStep(CRGB{canR, canG, canB}, canBrig, delayMs, modeFrame, canParam1, color2, brig2);
      break;
    }

    case 33: { // alternating block fade reset with color 2 as background
      if (modeRefresh) {
        modeFrame = 0;
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      modeFrame = alternatingBlockFadeStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, canParam1, color2, brig2);
      break;
    }

    case 34: { // alternating block fade no reset with color 2 as background
      if (modeRefresh) {
        modeRefresh = false;
      }
      if (color2Pending) {
        applyColor2ToOffPixels();
        color2Pending = false;
      }
      CRGB color2;
      uint8_t brig2 = 0;
      getColor2(color2, brig2);
      modeFrame = alternatingBlockFadeStep(CRGB{canR, canG, canB}, canBrig, canParam0, modeFrame, canParam1, color2, brig2);
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

    case 254: { // power-on init sequence
      if (modeRefresh) {
        powerPhase = 0;
        powerPhaseStartMs = millis();
        modeRefresh = false;
      }
      powerOnSequenceStep(powerPhase, powerPhaseStartMs);
      break;
    }

    default: //invalid case
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      break;
  }
}
