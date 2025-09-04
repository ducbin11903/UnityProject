#include <Arduino.h>
#include "ICM42688.h"
#include "I2Cdev.h"
#include <SimpleKalmanFilter.h>
// #include <WiFi.h>
// #include <WiFiUDP.h>
#include <Wire.h>
#include <HardwareSerial.h>

#define TXD1 17
#define RXD1 18
#define UART_BAUD 115200
#define H1 0xAA
#define H2 0x55
uint8_t txSeq = 0;
HardwareSerial MySerial(1);


float Raw_FingerAnalog[5];
float Map_FingerAngle[5];
int EMA_FingerAngle[5];
float Kalman_FingerAnalog[5];
int Finger_Pin[6] = {0, 6, 5, 4, 3, 2};
const float ALPHA = 0.5;  // Hệ số lọc EMA cho cảm biến uốn

SimpleKalmanFilter flexKalman1(10, 10, 0.01);
SimpleKalmanFilter flexKalman2(10, 10, 0.01);
SimpleKalmanFilter flexKalman3(10, 10, 0.01);
SimpleKalmanFilter flexKalman4(10, 10, 0.01);
SimpleKalmanFilter flexKalman5(10, 10, 0.01);

#pragma pack(push, 1) // Đảm bảo struct được đóng gói chặt, không có byte đệm
struct CombinedDataPacket {
  float w, x, y, z;        // Quaternion - 16 bytes
  float thumb_angle;       // Góc ngón cái - 4 bytes
  float index_angle;       // Góc ngón trỏ - 4 bytes
  float middle_angle;      // Góc ngón giữa - 4 bytes
  float ring_angle;        // Góc ngón áp út - 4 bytes
  float pinky_angle;       // Góc ngón út - 4 bytes
}; // Tổng cộng = 36 bytes
#pragma pack(pop)

// const char *ssid = "TEST";
// const char *password = "09112003";
// WiFiUDP udp;
// const char *udpAddress = "192.168.88.132";
// const int udpPortIMU = 1111;
// void setupWiFi();
// void checkWiFi();

void receiveAngle();
CombinedDataPacket mainPacket; // Biến toàn cục để lưu trữ gói tin

//const uint8_t SENSOR_ADDRESSES[] = {0x68, 0x69};

const uint8_t SENSOR_ADDRESSES = 0x69;

#define I2C_BUS          Wire                           // Define the I2C bus (Wire instance) you wish to use
I2Cdev                   i2c_0(&I2C_BUS);               // Instantiate the I2Cdev object and point to the desired I2C bus

bool clockIn = false;
float a12, a22, a31, a32, a33;
float pitch, yaw, roll;  
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
uint32_t delt_t = 0;                      // used to control display output rate
uint32_t sumCount = 0;                    // used to control display output rate
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
void Magdwick(float ax, float ay, float az, float gyrox, float gyroy, float gyroz);

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_15_625DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
      AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_50AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz, 
      AODR_1kHz, AODR_2kHz, AODR_4kHz, AODR_8kHz, AODR_16kHz, AODR_32kHz
      GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1kHz, GODR_2kHz, GODR_4kHz, GODR_8kHz, GODR_16kHz, GODR_32kHz
*/ 
uint8_t Ascale = AFS_8G, Gscale = GFS_2000DPS, AODR = AODR_500Hz, GODR = GODR_500Hz, aMode = aMode_LN, gMode = gMode_LN;
float aRes, gRes;                                                        // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0.0f, 0.0f,0.0f}, gyroBias[3] = {0.0f, 0.0f,0.0f}; // offset biases for the accel and gyro
int16_t accelDiff[3] = {0, 0, 0}, gyroDiff[3] = {0, 0, 0};               // difference betwee ST and normal values
float  STratio[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};          // self-test results for the accel and gyro
int16_t ICM42688Data[7];                                                 // Stores the 16-bit signed sensor output
float   Gtemperature;                                                    // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;                                            // variables to hold latest accel/gyro data values 

ICM42688 imu(&i2c_0, SENSOR_ADDRESSES);
void sendPacket(CombinedDataPacket &pkt);

void setup() {
  
  Serial.begin(230400);
  MySerial.begin(UART_BAUD, SERIAL_8N1, RXD1, TXD1);

  Wire.begin(7,8); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  

  //setupWiFi();
  // udp.begin(udpPortIMU);

  delay(100);

  imu.reset();  // software reset ICM42688 to default registers

  // set sensor resolutions for self test
  aRes = 4.0f/32768.0f;
  gRes = 250.0f/32768.0f;

  imu.selfTest(accelDiff, gyroDiff, STratio);
  
   // get sensor resolutions for user settings, only need to do this once
  aRes = imu.getAres(Ascale);
  gRes = imu.getGres(Gscale);

  imu.init(Ascale, Gscale, AODR, GODR, aMode, gMode, clockIn); // configure for basic accel/gyro data output  

  delay(1000);

  imu.offsetBias(accelBias, gyroBias);
  delay(1000);

}

void loop() {

  //checkWiFi();
  
  receiveAngle();

     for(uint8_t i = 0; i < 20; i++) {
      Now = micros();
      deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
      lastUpdate = Now;
      sum += deltat; // sum for averaging filter update rate
      sumCount++;
      
     imu.readData(ICM42688Data); // INT1 cleared on any read

   // Now we'll calculate the accleration value into actual g's
     ax = (float)ICM42688Data[1]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)ICM42688Data[2]*aRes - accelBias[1];   
     az = (float)ICM42688Data[3]*aRes - accelBias[2];  

   // Calculate the gyro value into actual degrees per second
     gx = (float)ICM42688Data[4]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
     gy = (float)ICM42688Data[5]*gRes - gyroBias[1];  
     gz = (float)ICM42688Data[6]*gRes - gyroBias[2]; 

    Magdwick(ax, ay, az, gx*pi/180.0f, gy*pi/180.0f, gz*pi/180.0f);
    //Gtemperature = ((float) ICM42688Data[0]) / 132.48f + 25.0f;
  }

  CombinedDataPacket packetToSend;
  packetToSend.w = q[0];
  packetToSend.x = q[1];
  packetToSend.y = q[2];
  packetToSend.z = q[3];

  // Suy luận chuyển động cho các ngón còn lại
  packetToSend.thumb_angle = EMA_FingerAngle[2]*0.8;
  packetToSend.index_angle = EMA_FingerAngle[2];
  packetToSend.middle_angle = EMA_FingerAngle[3];
  packetToSend.ring_angle = EMA_FingerAngle[4];
  packetToSend.pinky_angle = EMA_FingerAngle[5];
  
  // udp.beginPacket(udpAddress, udpPortIMU);
  // udp.write((byte*)&packetToSend, sizeof(packetToSend));
  // udp.endPacket();
  //Serial.write((uint8_t*)&packetToSend, sizeof(packetToSend));

  sendPacket(packetToSend);

  // Serial.println(packetToSend.w);
  // Serial.println(packetToSend.thumb_angle);

  delay(10);
  }

void Magdwick(float ax, float ay, float az, float gyrox, float gyroy, float gyroz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
            float norm;                                               // vector norm
            float f1, f2, f3;                                         // objetive funcyion elements
            float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
            float qDot1, qDot2, qDot3, qDot4;
            float hatDot1, hatDot2, hatDot3, hatDot4;
            float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

            // Auxiliary variables to avoid repeated arithmetic
            float _halfq1 = 0.5f * q1;
            float _halfq2 = 0.5f * q2;
            float _halfq3 = 0.5f * q3;
            float _halfq4 = 0.5f * q4;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;

            // Normalise accelerometer measurement
            norm = sqrtf(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;
            
            // Compute the objective function and Jacobian
            f1 = _2q2 * q4 - _2q1 * q3 - ax;
            f2 = _2q1 * q2 + _2q3 * q4 - ay;
            f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
            J_11or24 = _2q3;
            J_12or23 = _2q4;
            J_13or22 = _2q1;
            J_14or21 = _2q2;
            J_32 = 2.0f * J_14or21;
            J_33 = 2.0f * J_11or24;
          
            // Compute the gradient (matrix multiplication)
            hatDot1 = J_14or21 * f2 - J_11or24 * f1;
            hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
            hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
            hatDot4 = J_14or21 * f1 + J_11or24 * f2;
            
            // Normalize the gradient
            norm = sqrtf(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
            hatDot1 /= norm;
            hatDot2 /= norm;
            hatDot3 /= norm;
            hatDot4 /= norm;
            
            // Compute estimated gyroscope biases
            gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
            gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
            gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
            
            // Compute and remove gyroscope biases
            gbiasx += gerrx * deltat * zeta;
            gbiasy += gerry * deltat * zeta;
            gbiasz += gerrz * deltat * zeta;
            gyrox -= gbiasx;
            gyroy -= gbiasy;
            gyroz -= gbiasz;
            
            // Compute the quaternion derivative
            qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
            qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
            qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
            qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

            // Compute then integrate estimated quaternion derivative
            q1 += (qDot1 -(beta * hatDot1)) * deltat;
            q2 += (qDot2 -(beta * hatDot2)) * deltat;
            q3 += (qDot3 -(beta * hatDot3)) * deltat;
            q4 += (qDot4 -(beta * hatDot4)) * deltat;

            // Normalize the quaternion
            norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
      }   

void receiveAngle() {

Raw_FingerAnalog[1] = analogRead(Finger_Pin[1]);
Kalman_FingerAnalog[1] =(float)flexKalman1.updateEstimate(Raw_FingerAnalog[1]);
Map_FingerAngle[1] = (float)map(Raw_FingerAnalog[1], 2333, 3200, 0, 120);
Map_FingerAngle[1] = constrain(Map_FingerAngle[1], 0, 120);
EMA_FingerAngle[1] = (ALPHA * Map_FingerAngle[1]) + (1.0 - ALPHA) *  EMA_FingerAngle[1];
// Serial.print(EMA_FingerAngle[1]);
// Serial.print("  ");

Raw_FingerAnalog[2] = analogRead(Finger_Pin[2]);
Kalman_FingerAnalog[2] =(float)flexKalman2.updateEstimate(Raw_FingerAnalog[2]);
Map_FingerAngle[2] = (float)map(Raw_FingerAnalog[2], 2081, 2950, 0, 120);
Map_FingerAngle[2] = constrain(Map_FingerAngle[2], 0, 120);
EMA_FingerAngle[2] = (ALPHA * Map_FingerAngle[2]) + (1.0 - ALPHA) *  EMA_FingerAngle[2];
// Serial.print(EMA_FingerAngle[2]);
// Serial.print("  ");

Raw_FingerAnalog[3] = analogRead(Finger_Pin[3]);
Kalman_FingerAnalog[3] =(float)flexKalman3.updateEstimate(Raw_FingerAnalog[3]);
Map_FingerAngle[3] = (float)map(Raw_FingerAnalog[3], 2400, 3100, 0, 120);
Map_FingerAngle[3] = constrain(Map_FingerAngle[3], 0, 120);
EMA_FingerAngle[3] = (ALPHA* Map_FingerAngle[3]) + (1.0 - ALPHA) *  EMA_FingerAngle[3];
// Serial.print(EMA_FingerAngle[3]);
// Serial.print("  ");

Raw_FingerAnalog[4] = analogRead(Finger_Pin[4]);
Kalman_FingerAnalog[4] =(float)flexKalman4.updateEstimate(Raw_FingerAnalog[4]);
Map_FingerAngle[4] = (float)map(Raw_FingerAnalog[4], 2400, 3220, 0, 120);
Map_FingerAngle[4] = constrain(Map_FingerAngle[4], 0, 120);
EMA_FingerAngle[4] = (ALPHA * Map_FingerAngle[4]) + (1.0 - ALPHA) *  EMA_FingerAngle[4];
// Serial.print(EMA_FingerAngle[4]);
// Serial.print("  ");

Raw_FingerAnalog[5] = analogRead(Finger_Pin[5]);
Kalman_FingerAnalog[5] =(float)flexKalman5.updateEstimate(Raw_FingerAnalog[5]);
Map_FingerAngle[5] = (float)map(Raw_FingerAnalog[5], 2300, 3100, 0, 120);
Map_FingerAngle[5] = constrain(Map_FingerAngle[5], 0, 120);
EMA_FingerAngle[5] = (ALPHA * Map_FingerAngle[5]) + (1.0 - ALPHA) *  EMA_FingerAngle[5];
// Serial.println(EMA_FingerAngle[5]);
}

// void setupWiFi() {
//     Serial.print("🔄 Connecting to Wi-Fi...");
//     WiFi.disconnect(true);
//     WiFi.begin(ssid, password);
//     while (WiFi.status() != WL_CONNECTED) {
//         delay(500);
//         Serial.print(".");
//     }
//     Serial.println("\n✅ Wi-Fi connected!");
//     Serial.print("📡 IP Address: ");
//     Serial.println(WiFi.localIP());
// }

// void checkWiFi() {
//     if (WiFi.status() != WL_CONNECTED) {
//         Serial.println("⚠️ Wi-Fi lost. Reconnecting...");
//         WiFi.disconnect();
//         WiFi.reconnect();
//     }
// }

void sendPacket(CombinedDataPacket &pkt) {
  const uint8_t len = sizeof(pkt); // 36
  uint8_t buf[64];

  buf[0] = H1;
  buf[1] = H2;
  buf[2] = len;
  buf[3] = txSeq++;

  memcpy(&buf[4], &pkt, len);

  // checksum = sum(seq + payload)
  uint8_t checksum = buf[3];
  for (int i = 0; i < len; i++) checksum += buf[4 + i];
  buf[4 + len] = checksum;

  MySerial.write(buf, 5 + len); // header(2)+len(1)+seq(1)+payload+chk(1)
}