#include <Wire.h>
#include <TCA9548A.h>
#include "MPU6050_6Axis_MotionApps612.h"

// ── I2C & TCA config ────────────────────────────────────────
#define I2C_BUS_SDA 8
#define I2C_BUS_SCL 9
#define TCA_FREQ    400000

#define TCA_ADDR    0x71      // A0=1, A1=0, A2=0 → 0x71
#define TCA_CH_MPU  6         // MPU6050 on channel 7

TCA9548A TCA(TCA_ADDR);

// ── MPU6050 + DMP ───────────────────────────────────────────
MPU6050 IMU1(0x68);     // AD0=GND → address 0x68 ( proximal phalange )
MPU6050 IMU2(0x69);     // AD0=VCC -> address 0x69 ( Mid phalange)

bool     dmpReady1   = false;
uint8_t  devStatus1  = 0;
uint16_t packetSize1 = 0;
uint16_t fifoCount1  = 0;
uint8_t  fifoBuffer1[64];

bool     dmpReady2   = false;
uint8_t  devStatus2  = 0;
uint16_t packetSize2 = 0;
uint16_t fifoCount2  = 0;
uint8_t  fifoBuffer2[64];

Quaternion   q1;
VectorFloat  gravity1;
float        ypr1[3];

Quaternion   q2;
VectorFloat  gravity2;
float        ypr2[3];

void tcaSelectChannel(uint8_t ch) {
  TCA.closeAll();
  TCA.openChannel(ch);
}

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_BUS_SDA, I2C_BUS_SCL);
  Wire.setClock(TCA_FREQ);

  TCA.begin(Wire);

  // Route I2C through channel 7 to the MPU6050
  tcaSelectChannel(TCA_CH_MPU);

  IMU1.initialize();
  IMU2.initialize();
  Serial.println(IMU1.testConnection() ? "MPU6050 " + String(TCA_CH_MPU) + " Proximal OK" : "MPU6050 " + String(TCA_CH_MPU) + " Proximal FAIL");
  Serial.println(IMU2.testConnection() ? "MPU6050 " + String(TCA_CH_MPU) + " Mid OK" : "MPU6050 " + String(TCA_CH_MPU) + " Mid FAIL");


  Serial.print("WHOAMI: 0x");
  Serial.println(IMU1.getDeviceID(), HEX);
  Serial.println(IMU2.getDeviceID(), HEX);


  Serial.println(F("Initializing DMP..."));
  devStatus1 = IMU1.dmpInitialize();
  devStatus2 = IMU2.dmpInitialize();

  Serial.print("devStatus = " + String(devStatus1));
  Serial.print("devStatus = " + String(devStatus2));


  if (devStatus1 == 0) {
    IMU1.CalibrateAccel(6);
    IMU1.CalibrateGyro(6);
    IMU1.PrintActiveOffsets();

    Serial.println(F("Enabling DMP for proximal phalangael..."));
    IMU1.setDMPEnabled(true);

    packetSize1 = IMU1.dmpGetFIFOPacketSize();
    dmpReady1   = true;
  } 
  else 
  {
    Serial.print(F("mid phalangael DMP init failed (code "));
    Serial.print(devStatus1);
    Serial.println(F(")"));
  }

  if (devStatus2 == 0) {
    IMU2.CalibrateAccel(6);
    IMU2.CalibrateGyro(6);
    IMU2.PrintActiveOffsets();

    Serial.println(F("Enabling DMP for mid phalangael..."));
    IMU2.setDMPEnabled(true);

    packetSize2 = IMU2.dmpGetFIFOPacketSize();
    dmpReady2   = true;
  } 
  else 
  {
    Serial.print(F("mid phalangael DMP init failed (code "));
    Serial.print(devStatus2);
    Serial.println(F(")"));
  }
}

void loop() {
  if (!dmpReady1 || !dmpReady2) {
    delay(1000);
    return;
  }

  // Ensure channel 7 is still selected
  tcaSelectChannel(TCA_CH_MPU);

  fifoCount1 = IMU1.getFIFOCount();
  if (fifoCount1 < packetSize1) {
    delay(5);
    return;
  }

  if (fifoCount1 >= 1024) {      // FIFO overflow
    IMU1.resetFIFO();
    return;
  }

  IMU1.getFIFOBytes(fifoBuffer1, packetSize1);

  IMU1.dmpGetQuaternion(&q1, fifoBuffer1);
  IMU1.dmpGetGravity(&gravity1, &q1);
  IMU1.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);


  fifoCount2 = IMU2.getFIFOCount();
  if (fifoCount2 < packetSize2) {
    delay(5);
    return;
  }

  if (fifoCount2 >= 1024) {      // FIFO overflow
    IMU2.resetFIFO();
    return;
  }

  IMU2.getFIFOBytes(fifoBuffer2, packetSize2);

  IMU2.dmpGetQuaternion(&q2, fifoBuffer2);
  IMU2.dmpGetGravity(&gravity2, &q2);
  IMU2.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);



  Serial.print("ypr1[");
  Serial.print(ypr1[0] * 180.0 / M_PI); Serial.print(", ");
  Serial.print(ypr1[1] * 180.0 / M_PI); Serial.print(", ");
  Serial.print(ypr1[2] * 180.0 / M_PI); Serial.println("]");

  Serial.print("ypr2[");
  Serial.print(ypr2[0] * 180.0 / M_PI); Serial.print(", ");
  Serial.print(ypr2[1] * 180.0 / M_PI); Serial.print(", ");
  Serial.print(ypr2[2] * 180.0 / M_PI); Serial.println("]");

  delay(20);
}
