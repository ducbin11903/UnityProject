#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// THAY THẾ ĐỊA CHỈ MAC CỦA BO MẠCH NHẬN TẠI ĐÂY
uint8_t broadcastAddress[] = {0x48, 0x3B, 0xDA, 0x6E, 0x39, 0xC4};
//48:CA:43:96:F3:5C

// Cấu trúc dữ liệu để gửi đi. Phải giống hệt trên cả hai bo mạch.
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;

// Tạo một biến cấu trúc
struct_message myData;

esp_now_peer_info_t peerInfo;

// Callback function khi dữ liệu được gửi đi
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nTrang thai gui goi tin cuoi cung: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Giao thanh cong" : "Giao that bai");
}

void setup() {
  Serial.begin(230400);

  // Đặt thiết bị ở chế độ Station
  WiFi.mode(WIFI_STA);

  // Khởi tạo ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Loi khoi tao ESP-NOW");
    return;
  }

  // Đăng ký callback function cho việc gửi dữ liệu
  esp_now_register_send_cb(OnDataSent);

  // Đăng ký peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Thêm peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Khong the them peer");
    return;
  }
}

void loop() {
  // Thiết lập giá trị để gửi đi
  strcpy(myData.a, "DAY LA MOT CHUOI");
  myData.b = random(1, 20);
  myData.c = 1.2;
  myData.d = false;

  // Gửi tin nhắn qua ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Da gui voi thanh cong");
  }
  else {
    Serial.println("Loi khi gui du lieu");
  }
  delay(0);
}