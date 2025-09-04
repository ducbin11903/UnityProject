#include <Arduino.h>
#include <SimpleKalmanFilter.h>

// ================= FLEX SENSOR (1 ngón) ==================
int fingerPin = 5;  // Ngón trỏ -> Finger_Pin[2] = GPIO5 (theo mảng bạn đưa)
float rawValue = 0;
float kalmanValue = 0;

SimpleKalmanFilter flexKalman(5, 10, 0.01);

void setup() {
  Serial.begin(230400);
}

void loop() {
  // Đọc giá trị analog
  rawValue = analogRead(fingerPin);

  // Lọc Kalman
  kalmanValue = flexKalman.updateEstimate(rawValue);

  // In ra để vẽ đồ thị
  // Format CSV: raw,kalman
  Serial.print(rawValue);
  Serial.print(",");
  Serial.println(kalmanValue);

  delay(10);  // 100 Hz
}
