#include <Wire.h>
#include <TMD3725.h>
#include <Arduino.h>

TMD3725 tmd3725;
int reginfo[35];
int rawColor[9];

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("\n=== TMD3725 RAW RGB ===");

  if (!tmd3725.begin()) {
    Serial.println("Sensor NOT detected");
  }
  tmd3725.init(reginfo);
}

void loop() {
  tmd3725.get_optics_data(rawColor);  // fills 9 bytes: CLo,CHi,RLo,RHi,GLo,GHi,BLo,BHi,PDATA

  uint16_t clear = (uint16_t(rawColor[1]) << 8) | rawColor[0];
  uint16_t red   = (uint16_t(rawColor[3]) << 8) | rawColor[2];
  uint16_t green = (uint16_t(rawColor[5]) << 8) | rawColor[4];
  uint16_t blue  = (uint16_t(rawColor[7]) << 8) | rawColor[6];
  uint8_t  prox  = rawColor[8];

  // Print raw counts (always 0..65535 for RGB/C, 0..255 for prox)
  Serial.print("RAW  C:"); Serial.print(clear);
  Serial.print(" R:");     Serial.print(red);
  Serial.print(" G:");     Serial.print(green);
  Serial.print(" B:");     Serial.print(blue);
  Serial.print(" P:");     Serial.println(prox);

  // Step B – normalize to 0..255 to represent actual "colour"
  uint32_t sum = uint32_t(red) + green + blue;
  float rn=0, gn=0, bn=0;
  if (sum > 0) {
    rn = float(red)   / float(sum);
    gn = float(green) / float(sum);
    bn = float(blue)  / float(sum);
  }

  uint8_t r8 = uint8_t(rn * 255.0f + 0.5f);
  uint8_t g8 = uint8_t(gn * 255.0f + 0.5f);
  uint8_t b8 = uint8_t(bn * 255.0f + 0.5f);

  Serial.print("NORM RGB: ");
  Serial.print(int(r8)); Serial.print(", ");
  Serial.print(int(g8)); Serial.print(", ");
  Serial.println(int(b8));

  delay(50);
}
