#include "IMU_setup.h"
#include <Arduino.h>

void initFingerChannel(const FingerChannel& fc) {
  tcaSelectChannel(fc.tca_channel);

  Serial.println("Initialising " + fc.label);

  IMU_MID.initialize();
  IMU_PROX.initialize();

  Serial.println(IMU_PROX.testConnection() ? "MPU6050 " + fc.label + " Proximal OK" : "MPU6050 " + fc.label + " Proximal FAIL");
  Serial.println(IMU_MID.testConnection() ? "MPU6050 " + fc.label + " Mid OK"      : "MPU6050 " + fc.label + " Mid FAIL");

  Serial.print("WHOAMI1: 0x");
  Serial.println(IMU_MID.getDeviceID(), HEX);
  Serial.print("WHOAMI2: 0x");
  Serial.println(IMU_PROX.getDeviceID(), HEX);

  Serial.println(F("Initializing DMP..."));
  devStatus1 = IMU_MID.dmpInitialize();
  devStatus2 = IMU_PROX.dmpInitialize();

  Serial.print("devStatus1 = ");
  Serial.println(devStatus1);
  Serial.print("devStatus2 = ");
  Serial.println(devStatus2);

  if (devStatus1 == 0) {
    IMU_MID.CalibrateAccel(6);
    IMU_MID.CalibrateGyro(6);
    IMU_MID.PrintActiveOffsets();

    Serial.println(F("Enabling DMP for proximal phalangeal..."));
    IMU_MID.setDMPEnabled(true);

    packetSize1 = IMU_MID.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("proximal phalangeal DMP init failed (code "));
    Serial.print(devStatus1);
    Serial.println(F(")"));
    dmpReady1 = false;
  }

  if (devStatus2 == 0) {
    IMU_PROX.CalibrateAccel(6);
    IMU_PROX.CalibrateGyro(6);
    IMU_PROX.PrintActiveOffsets();

    Serial.println(F("Enabling DMP for mid phalangeal..."));
    IMU_PROX.setDMPEnabled(true);

    packetSize2 = IMU_PROX.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("mid phalangeal DMP init failed (code "));
    Serial.print(devStatus2);
    Serial.println(F(")"));
    dmpReady2 = false;
  }
}