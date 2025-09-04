#include <Arduino.h>
#include <HardwareSerial.h>

#define TXD2 17
#define RXD2 18
#define UART_BAUD 230400

HardwareSerial MySerial(1);

void setup() {
  Serial.begin(230400);
  MySerial.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2);

  Serial.println("UART Receiver started!");
}

void loop() {
  if (MySerial.available()) {
    String msg = MySerial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(msg);
  }
}
