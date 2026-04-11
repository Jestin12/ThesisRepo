#include "IMU_setup.h"
#include <Arduino.h>

void tcaSelectChannel(uint8_t ch) {
  TCA.closeAll();
  TCA.openChannel(ch);
}

void initFingerChannel(FingerChannel& fc, bool (&status)[4]) {
  tcaSelectChannel(fc.tca_channel);

  
  Serial.println("Initialising " + fc.label);

  IMU_MID.initialize();
  IMU_PROX.initialize();

  Serial.println(IMU_PROX.testConnection() ? "MPU6050 " + fc.label + " Proximal OK" : "MPU6050 " + fc.label + " Proximal FAIL");
  Serial.println(IMU_MID.testConnection() ? "MPU6050 " + fc.label + " Mid OK"      : "MPU6050 " + fc.label + " Mid FAIL");

  fc.IMU_PROX_EN  = IMU_PROX.testConnection() ? true : false;
  fc.IMU_MID_EN   = IMU_MID.testConnection() ? true : false;

  status[0] = IMU_MID.testConnection() ? true : false;
  status[2] = IMU_PROX.testConnection() ? true : false;

  // Serial.print("IMU_MID_ID: 0x");
  // Serial.println(IMU_MID.getDeviceID(), HEX);
  // Serial.print("IMU_PROX_ID: 0x");
  // Serial.println(IMU_PROX.getDeviceID(), HEX);

  // Serial.println(F("Initializing DMP..."));
  if (fc.IMU_MID_EN == true)
  {
    Serial.println(F("Initialising MID DMP"));
    devStatus1 = IMU_MID.dmpInitialize();
  }
  if (fc.IMU_PROX_EN == true)
  {
    Serial.println(F("Initialising PROX DMP"));
    devStatus2 = IMU_PROX.dmpInitialize();
  }

  
  

  // Serial.print("devStatus1 = ");
  // Serial.println(devStatus1);
  // Serial.print("devStatus2 = ");
  // Serial.println(devStatus2);

  if (fc.IMU_MID_EN == true) // If the IMU isn't connected, we're not going to bother to initialise DMP
  {
    if (devStatus1 == 0)
    {
      IMU_MID.CalibrateAccel(6);
      IMU_MID.CalibrateGyro(6);
      IMU_MID.PrintActiveOffsets();

      Serial.println(F("Enabling DMP for proximal phalangeal..."));
      IMU_MID.setDMPEnabled(true);

      packetSize1 = IMU_MID.dmpGetFIFOPacketSize();

      fc.IMU_MID_EN = true;
      status[1] = true;

    } 
    else 
    {
      Serial.print(F("proximal phalangeal DMP init failed (code "));
      Serial.print(devStatus1);
      Serial.println(F(")"));
      dmpReady1 = false;

      fc.IMU_MID_EN = false;
    }
  }

  if (fc.IMU_PROX_EN == true)  // If the IMU isn't connected, we're not going to bother to initialise DMP
  {
    if (devStatus2 == 0)
    {
      IMU_PROX.CalibrateAccel(6);
      IMU_PROX.CalibrateGyro(6);
      IMU_PROX.PrintActiveOffsets();

      Serial.println(F("Enabling DMP for mid phalangeal..."));
      IMU_PROX.setDMPEnabled(true);

      packetSize2 = IMU_PROX.dmpGetFIFOPacketSize();

      fc.IMU_PROX_EN = true;
      status[3] = true;
    } 
    else 
    {
      Serial.print(F("mid phalangeal DMP init failed (code "));
      Serial.print(devStatus2);
      Serial.println(F(")"));
      dmpReady2 = false;

      fc.IMU_PROX_EN = false;
    }
  }

}