// Globals.h
#pragma once
#include "MPU6050_6Axis_MotionApps612.h"
#include <TCA9548A.h>
#include <ArduinoJson.h>

struct FingerChannel {
  const int tca_channel;
  String label;
  const int adc_channel[2];
};
// shared globals
extern MPU6050 IMU_MID;
extern MPU6050 IMU_PROX;
extern TCA9548A TCA;
extern FingerChannel HandChannels[6];
// extern your JSON document from the other file, or declare it here.
extern DynamicJsonDocument DataPacket;

extern bool dmpReady1;
extern bool dmpReady2;
extern uint8_t  devStatus1;
extern uint8_t  devStatus2;
extern uint16_t packetSize1;
extern uint16_t packetSize2;


// any other globals you want shared