#include "common.h"
#include "IMU.h"

IMU fii_imu;
float quat[4];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin(400000);
  fii_imu.init(&Wire, 1);
}

void loop() {
  digitalToggle(LED_BUILTIN);
  fii_imu.read_data(quat);
  delay(10);
}