#include <Arduino.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  digitalToggle(LED_BUILTIN);
  Serial.println("Hello World\n");
  delay(500);
}