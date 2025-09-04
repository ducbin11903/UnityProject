#include <Arduino.h>
#include "ICM42688.h"
#include "I2Cdev.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

const char *ssid = "TEST";
const char *password = "09112003";
WiFiUDP udp;
const char *udpAddress = "192.168.88.132";

// ================= UDP =================
WiFiUDP udpRx;   // UDP để nhận ping
WiFiUDP udpTx;   // UDP để gửi quaternion

const int udpPortIMU = 1111;   // port gửi quaternion về PC
const int udpPortPing = 2222;  // port nhận ping từ PC

bool allowSend = false;        
unsigned long lastPingTime = 0;
const unsigned long pingTimeout = 1000; // 1 giây

// ================= DỮ LIỆU =================
#pragma pack(push, 1)
struct CombinedDataPacket {
  float w, x, y, z;
};
#pragma pack(pop)

CombinedDataPacket packetToSend;

// ================= IMU =================
const uint8_t SENSOR_ADDRESSES = 0x68;
#define I2C_BUS Wire
I2Cdev i2c_0(&I2C_BUS);
ICM42688 imu(&i2c_0, SENSOR_ADDRESSES);

float aRes, gRes;
float accelBias[3] = {0.0f}, gyroBias[3] = {0.0f};
int16_t ICM42688Data[7];
float ax, ay, az, gx, gy, gz; 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

float pi = 3.14159265358979323846f;
float GyroMeasError = pi * (40.0f / 180.0f);
float GyroMeasDrift = pi * (0.0f / 180.0f);
float beta = sqrtf(3.0f/4.0f) * GyroMeasError;
float zeta = sqrtf(3.0f/4.0f) * GyroMeasDrift;
float deltat = 0;
uint32_t lastUpdate = 0;
uint32_t Now = 0;

// ================= WIFI FUNC =================
void setupWiFi() {
    Serial.print("🔄 Connecting to Wi-Fi...");

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\n✅ Wi-Fi connected!");
    Serial.print("📡 ESP32 IP Address: ");
    Serial.println(WiFi.localIP());   // In ra IP được cấp

    udpRx.begin(udpPortPing);
    Serial.printf("📥 Listening for ping on UDP port %d\n", udpPortPing);
}


void checkWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("⚠️ Wi-Fi lost. Reconnecting...");
        WiFi.disconnect();
        WiFi.reconnect();
    }
}

void checkPing() {
    int packetSize = udpRx.parsePacket();
    if (packetSize) {
        char incoming[16];
        int len = udpRx.read(incoming, sizeof(incoming) - 1);
        if (len > 0) incoming[len] = '\0';
        Serial.printf("📩 Received ping: %s\n", incoming);
        lastPingTime = millis();
        allowSend = true;
    }
    if (millis() - lastPingTime > pingTimeout) {
        allowSend = false;
    }
}

void sendQuaternion(float w, float x, float y, float z) {
    Serial.printf("📤 Sent quaternion: [%.3f, %.3f, %.3f, %.3f]\n", w, x, y, z);
    if (!allowSend) return;
    packetToSend.w = w;
    packetToSend.x = x;
    packetToSend.y = y;
    packetToSend.z = z;
    udpTx.beginPacket(udpAddress, udpPortIMU);
    udpTx.write((byte*)&packetToSend, sizeof(packetToSend));
    udpTx.endPacket();  
}

// ================= MADGWICK =================
void M1(float ax, float ay, float az, float gx, float gy, float gz) {
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float f1, f2, f3;
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz;
    static float gbiasx=0, gbiasy=0, gbiasz=0;

    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;

    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;
    norm = 1.0f/norm;
    ax *= norm; ay *= norm; az *= norm;
    
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
          
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
    norm = sqrtf(hatDot1*hatDot1 + hatDot2*hatDot2 + hatDot3*hatDot3 + hatDot4*hatDot4);
    hatDot1 /= norm; hatDot2 /= norm; hatDot3 /= norm; hatDot4 /= norm;
          
    gerrx = _2q1*hatDot2 - _2q2*hatDot1 - _2q3*hatDot4 + _2q4*hatDot3;
    gerry = _2q1*hatDot3 + _2q2*hatDot4 - _2q3*hatDot1 - _2q4*hatDot2;
    gerrz = _2q1*hatDot4 - _2q2*hatDot3 + _2q3*hatDot2 - _2q4*hatDot1;
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx; gy -= gbiasy; gz -= gbiasz;
          
    qDot1 = -_halfq2*gx - _halfq3*gy - _halfq4*gz;
    qDot2 =  _halfq1*gx + _halfq3*gz - _halfq4*gy;
    qDot3 =  _halfq1*gy - _halfq2*gz + _halfq4*gx;
    qDot4 =  _halfq1*gz + _halfq2*gy - _halfq3*gx;
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;
    norm = sqrtf(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    norm = 1.0f/norm;
    q[0] = q1*norm; q[1] = q2*norm; q[2] = q3*norm; q[3] = q4*norm;
}

// ================= MAIN =================
void setup() {
    Serial.begin(230400);
    Wire.begin(17,18);

    setupWiFi();
    delay(100);

    imu.reset();
    aRes = imu.getAres(AFS_8G);
    gRes = imu.getGres(GFS_2000DPS);
    imu.init(AFS_8G, GFS_2000DPS, AODR_500Hz, GODR_500Hz, aMode_LN, gMode_LN, false);
    delay(1000);
    imu.offsetBias(accelBias, gyroBias);
    delay(1000);
}

void loop() {
    checkWiFi();
    checkPing();

    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f);
    lastUpdate = Now;

    imu.readData(ICM42688Data);
    ax = (float)ICM42688Data[1]*aRes - accelBias[0];
    ay = (float)ICM42688Data[2]*aRes - accelBias[1];
    az = (float)ICM42688Data[3]*aRes - accelBias[2];
    gx = (float)ICM42688Data[4]*gRes - gyroBias[0];
    gy = (float)ICM42688Data[5]*gRes - gyroBias[1];
    gz = (float)ICM42688Data[6]*gRes - gyroBias[2];

    M1(ax, ay, az, gx*pi/180.0f, gy*pi/180.0f, gz*pi/180.0f);

    sendQuaternion(q[0], q[1], q[2], q[3]);
}
