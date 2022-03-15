#include "common.h"
#include "IMU.h"

IMU FII_IMU;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin(400000);
  FII_IMU.init(&Wire, 1);
}

void loop() {
  digitalToggle(LED_BUILTIN);
  delay(500);
}