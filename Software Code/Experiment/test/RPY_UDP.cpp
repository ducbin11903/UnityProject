#include <Arduino.h>
#include "ICM42688.h"
#include "I2Cdev.h"
#include <WiFiUdp.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>


#pragma pack(push, 1) // Đảm bảo struct được đóng gói chặt, không có byte đệm
struct CombinedDataPacket {
  float roll;
  float pitch;
  float yaw;
}; 
#pragma pack(pop)

CombinedDataPacket mainPacket; // Biến toàn cục để lưu trữ gói tin
const char *ssid = "Codientu";
const char *password = "khongbiet";
WiFiUDP udp;
const char *udpAddress = "10.0.0.21";
const int udpPortIMU = 1111;
void setupWiFi();
void checkWiFi();

//const uint8_t SENSOR_ADDRESSES[] = {0x68, 0x69};

const uint8_t SENSOR_ADDRESSES = 0x68;

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
void M1(float ax, float ay, float az, float gyrox, float gyroy, float gyroz);

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

void setup() {
  
  Serial.begin(230400);
  //MySerial.begin(UART_BAUD_RATE);

  Wire.begin(17,18); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  

  setupWiFi();
  delay(100);

  //i2c_0.I2Cscan();

  imu.reset();  // software reset ICM42688 to default registers

  // set sensor resolutions for self test
  aRes = 4.0f/32768.0f;
  gRes = 250.0f/32768.0f;

  imu.selfTest(accelDiff, gyroDiff, STratio);

   /*Serial.println("Accel Self Test:");
   Serial.print("Ax diff: "); Serial.print(accelDiff[0]* aRes * 1000.0f); Serial.println(" mg");
   Serial.print("Ay diff: "); Serial.print(accelDiff[1]* aRes * 1000.0f); Serial.println(" mg");
   Serial.print("Az diff: "); Serial.print(accelDiff[2]* aRes * 1000.0f); Serial.println(" mg");
   Serial.println("Should be between 50 and 1200 mg");

   Serial.println("Gyro Self Test:");
   Serial.print("Gx diff: "); Serial.print(gyroDiff[0] * gRes); Serial.println(" dps");
   Serial.print("Gy diff: "); Serial.print(gyroDiff[1] * gRes); Serial.println(" dps");
   Serial.print("Gz diff: "); Serial.print(gyroDiff[2] * gRes); Serial.println(" dps");
   Serial.println("Should be > 60 dps");
 
   Serial.print("Ax ratio: "); Serial.print(STratio[1]*100.0f, 0); Serial.println(" %");
   Serial.print("Ay ratio: "); Serial.print(STratio[2]*100.0f, 0); Serial.println(" %");
   Serial.print("Az ratio: "); Serial.print(STratio[3]*100.0f, 0); Serial.println(" %");
   Serial.println("Should be between 50 and 150%");

   Serial.println("Gyro Self Test:");
   Serial.print("Gx ratio: "); Serial.print(STratio[4]*100.0f, 0); Serial.println(" %");
   Serial.print("Gy ratio: "); Serial.print(STratio[5]*100.0f, 0); Serial.println(" %");
   Serial.print("Gz ratio: "); Serial.print(STratio[6]*100.0f, 0); Serial.println(" %");
   Serial.println("Should be between 50 and 150%");
   delay(2000);*/
  
   // get sensor resolutions for user settings, only need to do this once
  aRes = imu.getAres(Ascale);
  gRes = imu.getGres(Gscale);

  imu.init(Ascale, Gscale, AODR, GODR, aMode, gMode, clockIn); // configure for basic accel/gyro data output  

   //Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
  delay(1000);

  imu.offsetBias(accelBias, gyroBias);
   //Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
   //Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
  delay(1000);

}

void loop() {

  checkWiFi();

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

    M1(ax, ay, az, gx*pi/180.0f, gy*pi/180.0f, gz*pi/180.0f);
    //Gtemperature = ((float) ICM42688Data[0]) / 132.48f + 25.0f;
    
  }

  a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
  a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
  a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
  a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
  a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  pitch = -asinf(a32);
  roll  = atan2f(a31, a33);
  yaw   = atan2f(a12, a22);
  pitch *= 180.0f / pi;
  yaw   *= 180.0f / pi; 
  yaw   -= 0.7f; // Declination at Viet Nam
  //if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
  roll  *= 180.0f / pi;                                                 
  
  CombinedDataPacket packetToSend;
  packetToSend.roll = roll;
  packetToSend.pitch = pitch;
  packetToSend.yaw = yaw;

  udp.beginPacket(udpAddress, udpPortIMU);
  udp.write((byte*)&packetToSend, sizeof(packetToSend));
  udp.endPacket();

  }

void M1(float ax, float ay, float az, float gyrox, float gyroy, float gyroz)
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

void setupWiFi() {
    Serial.print("🔄 Connecting to Wi-Fi...");
    WiFi.disconnect(true);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
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
