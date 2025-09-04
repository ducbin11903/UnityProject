#include <Arduino.h>
#include "WiFi.h"

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("Dia chi MAC cua ESP32-S3: ");
}

void loop(){
  Serial.println(WiFi.macAddress());
}
//D8:3B:DA:6E:39:C4