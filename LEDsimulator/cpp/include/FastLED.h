#pragma once
#include <cstdint>
#include <cmath>

struct CHSV {
  uint8_t h;
  uint8_t s;
  uint8_t v;
  CHSV(uint8_t hh, uint8_t ss, uint8_t vv) : h(hh), s(ss), v(vv) {}
};

struct CRGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;

  CRGB() : r(0), g(0), b(0) {}
  CRGB(uint8_t rr, uint8_t gg, uint8_t bb) : r(rr), g(gg), b(bb) {}

  CRGB& operator=(const CHSV& hsv) {
    float hf = (hsv.h / 255.0f) * 360.0f;
    float sf = hsv.s / 255.0f;
    float vf = hsv.v / 255.0f;

    int hi = static_cast<int>(hf / 60.0f) % 6;
    float f = (hf / 60.0f) - hi;
    float p = vf * (1.0f - sf);
    float q = vf * (1.0f - f * sf);
    float t = vf * (1.0f - (1.0f - f) * sf);

    float rr = 0.0f, gg = 0.0f, bb = 0.0f;
    switch (hi) {
      case 0: rr = vf; gg = t; bb = p; break;
      case 1: rr = q; gg = vf; bb = p; break;
      case 2: rr = p; gg = vf; bb = t; break;
      case 3: rr = p; gg = q; bb = vf; break;
      case 4: rr = t; gg = p; bb = vf; break;
      case 5: rr = vf; gg = p; bb = q; break;
      default: rr = vf; gg = t; bb = p; break;
    }

    r = static_cast<uint8_t>(rr * 255.0f + 0.5f);
    g = static_cast<uint8_t>(gg * 255.0f + 0.5f);
    b = static_cast<uint8_t>(bb * 255.0f + 0.5f);
    return *this;
  }

  void nscale8_video(uint8_t scale) {
    uint16_t s = static_cast<uint16_t>(scale) + 1;
    r = static_cast<uint8_t>((static_cast<uint16_t>(r) * s) >> 8);
    g = static_cast<uint8_t>((static_cast<uint16_t>(g) * s) >> 8);
    b = static_cast<uint8_t>((static_cast<uint16_t>(b) * s) >> 8);
  }

  bool operator==(const CRGB& other) const {
    return r == other.r && g == other.g && b == other.b;
  }

  static const CRGB Black;
  static const CRGB Red;
  static const CRGB Green;
  static const CRGB Blue;
  static const CRGB White;
};

inline CRGB blend(const CRGB& a, const CRGB& b, uint8_t amount) {
  uint16_t inv = 255 - amount;
  CRGB out;
  out.r = static_cast<uint8_t>((a.r * inv + b.r * amount + 127) / 255);
  out.g = static_cast<uint8_t>((a.g * inv + b.g * amount + 127) / 255);
  out.b = static_cast<uint8_t>((a.b * inv + b.b * amount + 127) / 255);
  return out;
}

inline void fill_solid(CRGB* leds, uint16_t num, const CRGB& color) {
  for (uint16_t i = 0; i < num; i++) {
    leds[i] = color;
  }
}

struct FastLED_t {
  void show() {}
};

extern FastLED_t FastLED;
