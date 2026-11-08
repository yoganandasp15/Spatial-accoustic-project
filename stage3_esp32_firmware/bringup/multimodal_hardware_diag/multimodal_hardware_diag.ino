#include <Arduino.h>

void setup() {
  Serial.begin(1152000);
  delay(1000);
  for (int i = 0; i < 5; i++) {
    Serial.printf("[TEST 1152000] Packet %d: ESP32 UART is alive and clear!\n", i);
    delay(100);
  }
}

void loop() {
  static uint32_t cnt = 0;
  Serial.printf("[STREAM 1152000] Counter: %u\n", cnt++);
  delay(200);
}
