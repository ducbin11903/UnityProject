#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <HardwareSerial.h>

// ================= WIFI CONFIG =================
const char *ssid = "Codientu";
const char *password = "khongbiet";
WiFiUDP udp;
const char *udpAddress = "10.0.0.30";
const int udpPortIMU = 1111;
// const char *ssid = "TEST";
// const char *password = "09112003";
// WiFiUDP udp;
// const char *udpAddress = "192.168.88.132";
// const int udpPortIMU = 1111;
// ================= UART CONFIG =================
#define RXD2 18
#define TXD2 17
#define UART_BAUD 115200
#define H1 0xAA
#define H2 0x55
HardwareSerial MySerial(1);
uint8_t state = 0;
uint8_t len = 0;
uint8_t idx = 0;
uint8_t checksum = 0;
uint8_t seq = 0;
uint8_t buf[64];
struct CombinedDataPacket {
  float w, x, y, z;
  float thumb_angle;
  float index_angle;
  float middle_angle;
  float ring_angle;
  float pinky_angle;
};

CombinedDataPacket packet;

void setupWiFi();
void checkWiFi();

void setup() {
  Serial.begin(230400);
  MySerial.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2);

  setupWiFi();
  udp.begin(udpPortIMU);

  Serial.println("ESP32 HardwareSerial -> UDP Forwarder started!");
}

void loop() {
checkWiFi();

    while (MySerial.available()) {
    uint8_t b = MySerial.read();

    switch (state) {
      case 0:
        if (b == H1) state = 1;
        break;
      case 1:
        if (b == H2) state = 2;
        else state = 0;
        break;
      case 2:
        len = b;
        idx = 0;
        state = 3;
        break;
      case 3:
        seq = b;
        checksum = b;
        state = 4;
        break;
      case 4:
        buf[idx++] = b;
        checksum += b;
        if (idx >= len) state = 5;
        break;
      case 5:
        if (b == checksum) {
          memcpy(&packet, buf, len);

          // ✅ Packet hợp lệ
          Serial.printf("Seq:%d Quat: %.4f, %.4f, %.4f, %.4f | Fingers: %.1f, %.1f, %.1f, %.1f, %.1f\n",
                        seq,
                        packet.w, packet.x, packet.y, packet.z,
                        packet.thumb_angle, packet.index_angle, packet.middle_angle,
                        packet.ring_angle, packet.pinky_angle);

          udp.beginPacket(udpAddress, udpPortIMU);
          udp.write((uint8_t*)&packet, sizeof(packet));
          udp.endPacket();
        }
        state = 0; // reset
        break;
    }
  }
delay(20); 
}


// ================= FUNC DEFINITION =================
void setupWiFi() {
    Serial.print("🔄 Connecting to Wi-Fi...");
    WiFi.disconnect(true);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(5000);
        Serial.print(".");
    }
    Serial.println("\n✅ Wi-Fi connected!");
    Serial.print("📡 IP Address: ");
    Serial.println(WiFi.localIP());
}

void checkWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("⚠️ Wi-Fi lost. Reconnecting...");
        WiFi.disconnect();
        WiFi.reconnect();
    }
}
