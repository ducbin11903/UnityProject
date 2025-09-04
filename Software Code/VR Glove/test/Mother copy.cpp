#include <Arduino.h>
int pins[] = {2, 3, 4, 5};  // các chân cần đọc

void setup() {
  Serial.begin(115200);
}

void loop() {
  for (int i = 0; i < 4; i++) {
    int value = analogRead(pins[i]);  // đọc ADC
    Serial.print(value);
    Serial.print(", ");
  }
  Serial.println("");
  delay(50);
}
