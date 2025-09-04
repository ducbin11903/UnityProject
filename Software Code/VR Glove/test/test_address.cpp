#include <Arduino.h>
#include <Wire.h>

#define SENSOR_ADDRESS 0x68      // Địa chỉ I2C của cảm biến
#define WHO_AM_I_REGISTER 0x75   // Địa chỉ thanh ghi WHO_AM_I

void setup() {
  Serial.begin(230400);
  while (!Serial); // Chờ Serial sẵn sàng

  Wire.begin(8,9);
  Wire.setClock(400000); // I2C Fast Mode

  Serial.println("--- I2C Communication Check ---");

  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(WHO_AM_I_REGISTER);
  byte error = Wire.endTransmission(false); // Dùng false để giữ kết nối

  if (error != 0) {
    Serial.print("Error finding device at 0x");
    Serial.print(SENSOR_ADDRESS, HEX);
    Serial.print(". I2C error code: ");
    Serial.println(error);
    Serial.println("Please check your wiring and the I2C address!");
    while(1); // Dừng chương trình ở đây
  }

  Wire.requestFrom(SENSOR_ADDRESS, 1);
  byte chip_id = Wire.read();

  Serial.print("Device Address: 0x");
  Serial.println(SENSOR_ADDRESS, HEX);
  Serial.print("WHO_AM_I Register (0x75) returned: 0x");
  Serial.println(chip_id, HEX);

  if (chip_id == 0x47) {
    Serial.println("\nSUCCESS! Communication with ICM-42688-P is working.");
    Serial.println("The problem is likely in the sensor configuration (init function).");
  } else {
    Serial.println("\nFAILED! Cannot communicate with the sensor correctly.");
    Serial.println("Please RE-CHECK your WIRING and I2C ADDRESS.");
  }
}

void loop() {
  // Không làm gì trong loop
  delay(1000);
}