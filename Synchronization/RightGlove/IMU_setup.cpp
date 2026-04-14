#pragma once
#include <Arduino.h>
#include "Globals.h"

void tcaSelectChannel(uint8_t ch) {
  TCA.closeAll();
  TCA.openChannel(ch);
}

void initFingerChannel(FingerChannel& fc, bool (&status)[4]) {
  tcaSelectChannel(fc.tca_channel);

  Serial.println("Initialising " + fc.label);

  IMU_MID.initialize();
  IMU_PROX.initialize();

  Serial.println(IMU_PROX.testConnection()
    ? "MPU6050 " + fc.label + " Proximal OK"
    : "MPU6050 " + fc.label + " Proximal FAIL");
  Serial.println(IMU_MID.testConnection()
    ? "MPU6050 " + fc.label + " Mid OK"
    : "MPU6050 " + fc.label + " Mid FAIL");

  fc.IMU_PROX_EN = IMU_PROX.testConnection();
  fc.IMU_MID_EN  = IMU_MID.testConnection();

  status[0] = fc.IMU_MID_EN;
  status[2] = fc.IMU_PROX_EN;

  Serial.println(F("Initializing DMP..."));
  devStatus1 = IMU_MID.dmpInitialize();
  devStatus2 = IMU_PROX.dmpInitialize();

  if (fc.IMU_MID_EN) {
    if (devStatus1 == 0) {
      IMU_MID.CalibrateAccel(6);
      IMU_MID.CalibrateGyro(6);
      IMU_MID.PrintActiveOffsets();
      Serial.println(F("Enabling DMP for MID IMU..."));
      IMU_MID.setDMPEnabled(true);
      packetSize1 = IMU_MID.dmpGetFIFOPacketSize();
      dmpReady1 = true;
      status[1] = true;
    } else {
      Serial.print(F("MID DMP init failed (code "));
      Serial.print(devStatus1);
      Serial.println(F(")"));
      dmpReady1 = false;
      fc.IMU_MID_EN = false;
    }
  }

  if (fc.IMU_PROX_EN) {
    if (devStatus2 == 0) {
      IMU_PROX.CalibrateAccel(6);
      IMU_PROX.CalibrateGyro(6);
      IMU_PROX.PrintActiveOffsets();
      Serial.println(F("Enabling DMP for PROX IMU..."));
      IMU_PROX.setDMPEnabled(true);
      packetSize2 = IMU_PROX.dmpGetFIFOPacketSize();
      dmpReady2 = true;
      status[3] = true;
    } else {
      Serial.print(F("PROX DMP init failed (code "));
      Serial.print(devStatus2);
      Serial.println(F(")"));
      dmpReady2 = false;
      fc.IMU_PROX_EN = false;
    }
  }
}