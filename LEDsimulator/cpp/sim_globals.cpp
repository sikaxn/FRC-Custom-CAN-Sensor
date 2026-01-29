#include <vector>
#include <cstdint>
#include "FastLED.h"

// Globals expected by mode/pattern code
uint16_t NUM_LEDS = 60;
CRGB* leds = nullptr;

volatile uint8_t canMode = 0;
volatile uint8_t canR = 255;
volatile uint8_t canG = 0;
volatile uint8_t canB = 0;
volatile uint8_t canBrig = 128;
volatile bool canOnOff = true;
volatile uint8_t canParam0 = 0;
volatile uint8_t canParam1 = 0;

volatile uint8_t canR2 = 0;
volatile uint8_t canG2 = 0;
volatile uint8_t canB2 = 0;
volatile uint8_t canBrig2 = 0;
volatile uint8_t canOnOff2 = 0;

volatile bool modeRefresh = true;

volatile bool customSeen = false;
volatile uint16_t customPix = 0;
volatile uint8_t cR = 0;
volatile uint8_t cG = 0;
volatile uint8_t cB = 0;

static std::vector<CRGB> g_leds;

void sim_set_num_leds(uint16_t count) {
  NUM_LEDS = count;
  g_leds.assign(count, CRGB::Black);
  leds = g_leds.data();
}

CRGB* sim_leds_data() {
  return leds;
}

uint16_t sim_leds_count() {
  return NUM_LEDS;
}

void sim_get_leds_copy(std::vector<CRGB>& out) {
  out.assign(g_leds.begin(), g_leds.end());
}

void sim_set_led(uint16_t index, const CRGB& color) {
  if (index < NUM_LEDS && leds) {
    leds[index] = color;
  }
}

CRGB sim_get_led(uint16_t index) {
  if (index < NUM_LEDS && leds) {
    return leds[index];
  }
  return CRGB::Black;
}
