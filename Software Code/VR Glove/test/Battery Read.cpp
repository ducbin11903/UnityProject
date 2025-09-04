#include <Arduino.h>
#define PIN_BATTERY 10   // ADC pin đọc điện áp (GPIO34)

// Hệ số chia áp
const float R1 = 20000.0; // ohm
const float R2 = 4700.0;  // ohm

// Điện áp tham chiếu ADC ESP32 (3.3V, 12-bit ADC = 4095)
const float ADC_MAX = 4095.0;
const float VREF = 3.3;

float readBatteryVoltage() {
  int raw = analogRead(PIN_BATTERY);
  float v_adc = (raw / ADC_MAX) * VREF;
  float v_bat = v_adc * (R1 + R2) / R2;  // công thức đảo
  return v_bat;
}

int batteryPercent(float v_bat) {
  // LiPo 1 cell: 4.20V (100%) → 3.30V (0%)
  if (v_bat >= 4.2) return 100;
  if (v_bat <= 3.3) return 0;
  return (int)((v_bat - 3.3) * 100 / (4.2 - 3.3));
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // ESP32: 12-bit
}

void loop() {
  float v_bat = readBatteryVoltage();
  int percent = batteryPercent(v_bat);

  Serial.print("Battery Voltage: ");
  Serial.print(v_bat, 2);
  Serial.print(" V  |  ");
  Serial.print("Battery: ");
  Serial.print(percent);
  Serial.println(" %");

  delay(1000);
}
