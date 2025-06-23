#include "mode.h"
#include "pattern.h"

extern volatile uint8_t canMode, canR, canG, canB, canBrig, canParam0, canParam1;
extern volatile bool canOnOff;

extern volatile bool modeRefresh;
extern CRGB leds[];
extern const uint16_t NUM_LEDS;

static uint16_t modeFrame = 0;

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



  //ADD Your Own Mode Here

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
