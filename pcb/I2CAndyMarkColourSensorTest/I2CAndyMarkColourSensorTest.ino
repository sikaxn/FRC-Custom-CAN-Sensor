#include <Wire.h>
#include <TMD3725.h>
#include <Arduino.h>

TMD3725 tmd3725;
optics_val color;
int reginfo[35];
int rawColor[9];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("\n=== TMD3725 Unsigned 16-bit Clamp Output ===");
  tmd3725.init(reginfo);
}

void loop() {
  tmd3725.get_all_data(reginfo);
  tmd3725.get_optics_data(rawColor);

  color = tmd3725.get_calib_color(reginfo);

  // ✅ Convert to uint16 and clamp 0–65535
  uint16_t red   = constrain((int32_t)color.red,   0, 65535);
  uint16_t green = constrain((int32_t)color.green, 0, 65535);
  uint16_t blue  = constrain((int32_t)color.blue,  0, 65535);
  uint16_t clear = constrain((int32_t)color.clear, 0, 65535);

  // ✅ Proximity already 1-byte 0–255, but print decimal
  uint8_t prx = rawColor[8];

  // Print clean decimal only
  Serial.println("CLR   RED   GRN   BLU   PRX");
  Serial.printf("%u %u %u %u %u\n\n",
        (uint16_t)clear,
        (uint16_t)red,
        (uint16_t)green,
        (uint16_t)blue,
        (uint8_t)prx
  );

  delay(1000);
}
