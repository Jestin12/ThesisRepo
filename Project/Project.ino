#include <Wire.h>
// #include <TCA9548A.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include <ArduinoJson.h>

#include "Globals.h"
#include "IMU_setup.h"

#include "CreateJson.h"

// ── I2C & TCA config ────────────────────────────────────────
#define I2C_BUS_SDA 8
#define I2C_BUS_SCL 9
#define TCA_FREQ    400000

#define TCA_ADDR    0x71      // A0=1, A1=0, A2=0 → 0x71

#define TCA_CH_PALM  7         // MPU6050 on channel 7 (PALM)
#define TCA_CH_THUMB 6        
#define TCA_CH_INDEX 5         
#define TCA_CH_MIDDLE 4         
#define TCA_CH_RING 3         
#define TCA_CH_PINKY 2        


TCA9548A TCA(TCA_ADDR);

// ── MPU6050 + DMP ───────────────────────────────────────────
MPU6050 IMU_MID(0x68);     // AD0=GND → address 0x68 ( proximal phalange )
MPU6050 IMU_PROX(0x69);     // AD0=VCC -> address 0x69 ( Mid phalange)

// struct FingerChannel {
//   const int tca_channel;
//   String label;
//   const int adc_channel[2];
// };

FingerChannel HandChannels[6] = {
  {TCA_CH_PALM, "Palm", {}},
  {TCA_CH_THUMB, "Thumb", {2,3}},
  {TCA_CH_INDEX, "Index", {4,5}},
  {TCA_CH_MIDDLE, "Middle", {6,13}},
  {TCA_CH_RING, "Ring", {12,11}},
  {TCA_CH_PINKY, "Pinky", {10,7}}
};

// unit8_t TCA_Channels[6] = {TCA_CH_PALM, TCA_CH_THUMB, TCA_CH_INDEX, TCA_CH_MIDDLE, TCA_CH_RING, TCA_CH_PINKY};

bool     dmpReady1   = true;
uint8_t  devStatus1  = 0;
uint16_t packetSize1 = 0;
uint16_t fifoCount1  = 0;
uint8_t  fifoBuffer1[64];

bool     dmpReady2   = true;
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

DynamicJsonDocument DataPacket(1024);


void setup() {
  Serial.begin(115200);

  
  JsonObject fingerData = DataPacket.createNestedObject("Data");

  buildFingerData(fingerData);

  Wire.begin(I2C_BUS_SDA, I2C_BUS_SCL);
  Wire.setClock(TCA_FREQ);

  TCA.begin(Wire);

  analogReadResolution(12);

  DataPacket["Hand"] = "Left";
  DataPacket["Time"] = NULL;

  
  // prints nicely formatted JSON:
  serializeJsonPretty(DataPacket, Serial);
  Serial.println();
  // Route I2C through channel 7 to the MPU6050s

  for (int i = 0; i < int(sizeof(HandChannels)/sizeof(HandChannels[0])); i++)
  {
    initFingerChannel(HandChannels[i]);
  }
    
}

void loop() {

  Serial.println("working");

  // if (!dmpReady1 || !dmpReady2) {
  //   delay(1000);
  //   return;
  // }

  for (int i = 0; i < int(sizeof(HandChannels)/sizeof(HandChannels[0])); i++)
  {
    tcaSelectChannel(HandChannels[i].tca_channel);

    // ---- IMU_MID ----
    fifoCount1 = IMU_MID.getFIFOCount();
    if (fifoCount1 >= 1024) {
      IMU_MID.resetFIFO();
    } else if (fifoCount1 >= packetSize1) {
      IMU_MID.getFIFOBytes(fifoBuffer1, packetSize1);
      IMU_MID.dmpGetQuaternion(&q1, fifoBuffer1);
      IMU_MID.dmpGetGravity(&gravity1, &q1);
      IMU_MID.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
    }

    // ---- IMU_PROX ----
    fifoCount2 = IMU_PROX.getFIFOCount();
    if (fifoCount2 >= 1024) {
      IMU_PROX.resetFIFO();
    } else if (fifoCount2 >= packetSize2) {
      IMU_PROX.getFIFOBytes(fifoBuffer2, packetSize2);
      IMU_PROX.dmpGetQuaternion(&q2, fifoBuffer2);
      IMU_PROX.dmpGetGravity(&gravity2, &q2);
      IMU_PROX.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
    }

    int MCP_flex = analogRead((HandChannels[i].label == "Palm") ? NULL : HandChannels[i].adc_channel[1]);
    int PIP_flex = analogRead((HandChannels[i].label == "Palm") ? NULL : HandChannels[i].adc_channel[2]);

    DataPacket["Data"][HandChannels[i].label]["flex_mcp"]     = MCP_flex;
    DataPacket["Data"][HandChannels[i].label]["flex_pip"]     = PIP_flex;
    DataPacket["Data"][HandChannels[i].label]["yaw_prox"]     = ypr1[0];
    DataPacket["Data"][HandChannels[i].label]["pitch_prox"]   = ypr1[1];
    DataPacket["Data"][HandChannels[i].label]["roll_prox"]    = ypr1[2];
    DataPacket["Data"][HandChannels[i].label]["ax_prox"]      = NULL;
    DataPacket["Data"][HandChannels[i].label]["ay_prox"]      = NULL;
    DataPacket["Data"][HandChannels[i].label]["az_prox"]      = NULL;
    DataPacket["Data"][HandChannels[i].label]["yaw_mid"]      = ypr2[0];
    DataPacket["Data"][HandChannels[i].label]["pitch_mid"]    = ypr2[1];
    DataPacket["Data"][HandChannels[i].label]["roll_mid"]     = ypr2[2];
    DataPacket["Data"][HandChannels[i].label]["ax_mid"]       = NULL;
    DataPacket["Data"][HandChannels[i].label]["ay_mid"]       = NULL;
    DataPacket["Data"][HandChannels[i].label]["az_mid"]       = NULL;
  }

  delay(20);
}