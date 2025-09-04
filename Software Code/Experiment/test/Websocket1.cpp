#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <WebSocketsServer.h>
#include "I2Cdev.h"
#include "ICM42688.h"


const char* default_ssid = "Supreme Shoulder";
const char* default_password = "12345678";

WebServer server(80);
WebSocketsServer WebSocketServer2(2222);
int numNetworks = 0;

bool calibrateNow = false;

int i = 0;
const uint8_t SENSOR_ADDRESSES[] = {0x68, 0x69};

void M1(float ax, float ay, float az, float gyrox, float gyroy, float gyroz);
void calibrateIMU();

#define I2C_BUS          Wire                           // Define the I2C bus (Wire instance) you wish to use
I2Cdev                   i2c_0(&I2C_BUS);               // Instantiate the I2Cdev object and point to the desired I2C bus
ICM42688 imu(&i2c_0, SENSOR_ADDRESSES[0]);

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
uint8_t Ascale = AFS_8G, Gscale = GFS_2000DPS, AODR = AODR_500Hz, GODR = GODR_500Hz, aMode = aMode_LN, gMode = gMode_LN;
float aRes, gRes;                                                        // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0.0f, 0.0f,0.0f}, gyroBias[3] = {0.0f, 0.0f,0.0f}; // offset biases for the accel and gyro
int16_t accelDiff[3] = {0, 0, 0}, gyroDiff[3] = {0, 0, 0};               // difference betwee ST and normal values
float  STratio[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};          // self-test results for the accel and gyro
int16_t ICM42688Data[7];                                                 // Stores the 16-bit signed sensor output
float   Gtemperature;                                                    // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;                                            // variables to hold latest accel/gyro data values 

void handleScanPage() {
  String html = "<html><body>";
  html += "<h2>Select Wi-Fi Network</h2>";
  html += "<form action='/connect' method='POST'>";
  html += "SSID: <select name='ssid'>";
  
  for (int i = 0; i < numNetworks; i++) {
    String ssid = WiFi.SSID(i);
    html += "<option value='" + ssid + "'>" + ssid + "</option>";
  }
  
  html += "</select><br>";
  html += "Password: <input type='password' name='password'><br>";
  html += "<input type='submit' value='Connect'>";
  html += "</form>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}


void handleConnect() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");

  if (ssid == "" || password == "") {
    server.send(400, "text/html", "SSID and password cannot be empty.");
    return;
  }

  WiFi.begin(ssid.c_str(), password.c_str());
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(1000);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    String html = "<html><body>";
    html += "<h2>Wi-Fi Connected Successfully!</h2>";
    html += "<p>Device IP Address: " + WiFi.localIP().toString() + "</p>";
    html += "</body></html>";
    server.send(200, "text/html", html);

    WebSocketServer2.begin();
    Serial.println("WebSocket server is now running on port 2222");

  } else {
    server.send(500, "text/html", "Failed to connect to Wi-Fi.");
  }
}

void setup() {
  Serial.begin(230400);
  pinMode(2, OUTPUT);
  Wire.begin(17, 18); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(100);
  //i2c_0.I2Cscan();
  calibrateIMU();

  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to initialize SPIFFS.");
    return;
}

WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(default_ssid, default_password);
  Serial.print("Access Point IP Address: ");
  Serial.println(WiFi.softAPIP());
  delay(100);

  digitalWrite(2, HIGH);
  delay(5000);
  digitalWrite(2, LOW);

  Serial.println("Scanning for networks...");
  numNetworks = WiFi.scanNetworks();
  Serial.println("Scan complete.");

  server.on("/", HTTP_GET, handleScanPage);
  server.on("/connect", HTTP_POST, handleConnect);
  server.begin();

  WebSocketServer2.onEvent([](uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
      String message = String((char*)payload);
      if (message == "calibrate") {
        calibrateNow = true;
        WebSocketServer2.broadcastTXT("Calibrating Shoulder");
        Serial.println("Calibration request received from IMU WebSocket!");
      }
    }
  });
}

void loop() {
    server.handleClient();
    WebSocketServer2.loop();
    i = 0;
    String data = "";
    imu.readData(ICM42688Data); // INT1 cleared on any read
   
   // Now we'll calculate the accleration value into actual g's
     ax = (float)ICM42688Data[1]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)ICM42688Data[2]*aRes - accelBias[1];   
     az = (float)ICM42688Data[3]*aRes - accelBias[2];  

   // Calculate the gyro value into actual degrees per second
     gx = (float)ICM42688Data[4]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
     gy = (float)ICM42688Data[5]*gRes - gyroBias[1];  
     gz = (float)ICM42688Data[6]*gRes - gyroBias[2]; 

     for(uint8_t i = 0; i < 20; i++) {
      Now = micros();
      deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
      lastUpdate = Now;
      sum += deltat; // sum for averaging filter update rate
      sumCount++;

    M1(ax, ay, az, gx*pi/180.0f, gy*pi/180.0f, gz*pi/180.0f);
    //Gtemperature = ((float) ICM42688Data[0]) / 132.48f + 25.0f;
  }
    //192.168.88.124
    /*Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");*/
    
  // Serial.print(">qw = :"); 
  // Serial.print(q[0]);
  // Serial.print(",");
  // Serial.print(">qx = :"); 
  // Serial.print(q[1]); 
  // Serial.print(",");
  // Serial.print(">qy = :"); 
  // Serial.print(q[2]);
  // Serial.print(","); 
  // Serial.print(">qz = :"); 
  // Serial.println(q[3]); 
 
  // Serial.print("w"); Serial.print(q[0], 4);
  // Serial.print("a"); Serial.print(q[2], 4);
  // Serial.print("b"); Serial.print(q[1], 4);
  // Serial.print("c"); Serial.println(q[3], 4);

    data +=  String(q[0], 4) + "," + 
                String(q[1], 4) + "," + 
                String(q[2], 4) + "," + 
                String(q[3], 4);
    WebSocketServer2.sendTXT(0, data);
    Serial.println(data);

    if(calibrateNow)
    {
      calibrateNow = false;
      calibrateIMU();
      WebSocketServer2.broadcastTXT("Shoulder Done");
}
}

void calibrateIMU(){

  Serial.println("🚀 Bat dau qua trinh hieu chinh IMU...");

  imu.reset();
  aRes = 4.0f / 32768.0f;
  gRes = 250.0f / 32768.0f;

  imu.selfTest(accelDiff, gyroDiff, STratio);
  delay(1000); // Doi mot chut de on dinh
  aRes = imu.getAres(Ascale);
  gRes = imu.getGres(Gscale);
  imu.init(Ascale, Gscale, AODR, GODR, aMode, gMode, clockIn);
  delay(1000);
  imu.offsetBias(accelBias, gyroBias);
  delay(1000);
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