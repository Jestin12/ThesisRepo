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
MPU6050 IMU(0x69);                  // AD0=GND → address 0x68

bool     dmpReady   = false;
uint8_t  devStatus  = 0;
uint16_t packetSize = 0;
uint16_t fifoCount  = 0;
uint8_t  fifoBuffer[64];

Quaternion   q;
VectorFloat  gravity;
float        ypr[3];

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

  IMU.initialize();
  Serial.println(IMU.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");

  Serial.print("WHOAMI: 0x");
  Serial.println(IMU.getDeviceID(), HEX);

  Serial.println(F("Initializing DMP..."));
  devStatus = IMU.dmpInitialize();
  Serial.print("devStatus = ");
  Serial.println(devStatus);

  if (devStatus == 0) {
    IMU.CalibrateAccel(6);
    IMU.CalibrateGyro(6);
    IMU.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    IMU.setDMPEnabled(true);

    packetSize = IMU.dmpGetFIFOPacketSize();
    dmpReady   = true;
  } else {
    Serial.print(F("DMP init failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  if (!dmpReady) {
    delay(1000);
    return;
  }

  // Ensure channel 7 is still selected
  tcaSelectChannel(TCA_CH_MPU);

  fifoCount = IMU.getFIFOCount();
  if (fifoCount < packetSize) {
    delay(5);
    return;
  }

  if (fifoCount >= 1024) {      // FIFO overflow
    IMU.resetFIFO();
    return;
  }

  IMU.getFIFOBytes(fifoBuffer, packetSize);

  IMU.dmpGetQuaternion(&q, fifoBuffer);
  IMU.dmpGetGravity(&gravity, &q);
  IMU.dmpGetYawPitchRoll(ypr, &q, &gravity);

  Serial.print("ypr[");
  Serial.print(ypr[0] * 180.0 / M_PI); Serial.print(", ");
  Serial.print(ypr[1] * 180.0 / M_PI); Serial.print(", ");
  Serial.print(ypr[2] * 180.0 / M_PI); Serial.println("]");

  delay(20);
}
