// Globals.h
#pragma once
#include "MPU6050_6Axis_MotionApps612.h"
#include <TCA9548A.h>



struct FingerChannel;
// shared globals
extern MPU6050 IMU_MID;
extern MPU6050 IMU_PROX;
extern TCA9548A TCA;
extern FingerChannel HandChannels

extern bool dmpReady1;
extern bool dmpReady2;


// any other globals you want shared