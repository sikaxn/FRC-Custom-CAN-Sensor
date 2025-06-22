#include "mode.h"
#include "pattern.h"

extern volatile uint8_t canMode, canR, canG, canB, canBrig, canParam0, canParam1;
extern volatile bool canOnOff;

extern volatile bool modeRefresh;
extern CRGB leds[];
extern const uint16_t NUM_LEDS;

// Frame index for animations
static uint16_t wipe_index = 0;
static uint16_t rainbow_index = 0;

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

    case 2: {  // Color wipe (non-blocking)
      static uint16_t wipe_index = 0;
      if (modeRefresh) {
        fill_solid(leds, NUM_LEDS, CRGB::Black);  // clear first
        FastLED.show();
        wipe_index = 0;
        modeRefresh = false;
      }
      wipe_index = colorWipeStep(CRGB{canR, canG, canB}, canParam0, wipe_index);
      break;
    }
    
    //case N:
    //Your own mode goes here



    case 3: {  // Rainbow
      if (modeRefresh) {
        rainbow_index = 0;
        modeRefresh = false;
      }
      rainbow_index = rainbowStep(canParam0, rainbow_index);
      break;
    }

    case 255: {
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


    default:
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      break;
  }
}
