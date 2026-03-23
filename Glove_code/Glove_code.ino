/**
 * MPU6050 IMU Interface — ESP32-S3
 *
 * Reads raw accelerometer and gyroscope data from a single MPU6050
 * and outputs JSON over USB Serial.
 *
 * Wiring:
 *   MPU6050 SDA  ->  ESP32-S3 GPIO 11
 *   MPU6050 SCL  ->  ESP32-S3 GPIO 10
 *   MPU6050 VCC  ->  3.3V
 *   MPU6050 GND  ->  GND
 *   MPU6050 AD0  ->  GND  (I2C address = 0x68)
 *
 * Library: Adafruit MPU6050  (install via Library Manager)
 * Depends: Adafruit Unified Sensor, Adafruit BusIO
 */

#include <Wire.h>
#include <I2Cdev.h>
// #include <MPU6050.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include <TCA9548A.h>



// ── Configuration ─────────────────────────────────────────────────────────────


#define TCA_FREQ 400000

#define I2C_BUS_SDA 45
#define I2C_BUS_SCL 41

#define TCA1_ADDR 0x72
#define TCA2_ADDR 0x74

TCA9548A TCA1(TCA1_ADDR);
TCA9548A TCA2(TCA2_ADDR);

MPU6050 IMU;

// DMP-related (only needed if you use DMP)
bool     dmpReady    = false;
uint8_t  devStatus   = 0;
uint16_t packetSize  = 0;
uint16_t fifoCount   = 0;
uint8_t  fifoBuffer[64];

Quaternion q;        // [w, x, y, z]
VectorFloat gravity; // [x, y, z]
float ypr[3];        // [yaw, pitch, roll] in radians

void setup()
{
  Serial.begin(115200);

  Wire.begin(I2C_BUS_SDA, I2C_BUS_SCL);
  Wire.setClock(TCA_FREQ);

  TCA1.begin(Wire);
  TCA2.begin(Wire);

  selectMPU(0);  // uses muxDisableAll/muxSelect from MUX_Functions.ino[file:115]

  IMU.initialize();
  Serial.println(IMU.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");

  // Serial.println(F("Initializing DMP..."));
  // devStatus = IMU.dmpInitialize();

  // if (devStatus == 0) 
  // {
  //   // Calibration helpers (built into Jeff’s fork)
  //   IMU.CalibrateAccel(6);                    // optional but recommended[web:77]
  //   IMU.CalibrateGyro(6);                     // optional but recommended[web:77]
  //   IMU.PrintActiveOffsets();                 // diagnostic[web:77]

  //   Serial.println(F("Enabling DMP..."));
  //   IMU.setDMPEnabled(true);                  // turn on DMP[web:77]

  //   // Get expected packet size for DMP
  //   packetSize = IMU.dmpGetFIFOPacketSize();  // usually 42 bytes[web:77]
  //   dmpReady = true;
  // } 
  // else 
  // {
  //   Serial.print(F("DMP init failed (code "));
  //   Serial.print(devStatus);
  //   Serial.println(F(")"));
  // }
}

void loop()
{

  delay(50);
}
