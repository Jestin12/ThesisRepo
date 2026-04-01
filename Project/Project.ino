#include <Wire.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include <ArduinoJson.h>

#include "Globals.h"
#include "IMU_setup.h"
#include "CreateJson.h"

// ── BLE includes ───────────────────────────────────────────
#include "BLE.h"

// ── WIFI includes ───────────────────────────────────────────
#include "Net.h"

// ── I2C & TCA config ────────────────────────────────────────
#define I2C_BUS_SDA 8
#define I2C_BUS_SCL 9
#define TCA_FREQ    400000

#define TCA_ADDR    0x71

#define TCA_CH_PALM   7
#define TCA_CH_THUMB  6
#define TCA_CH_INDEX  5
#define TCA_CH_MIDDLE 4
#define TCA_CH_RING   3
#define TCA_CH_PINKY  2

TCA9548A TCA(TCA_ADDR);

// ── MPU6050 + DMP ───────────────────────────────────────────
MPU6050 IMU_MID(0x68);
MPU6050 IMU_PROX(0x69);

FingerChannel HandChannels[6] = {
  {TCA_CH_PALM,  "Palm",  {}},
  {TCA_CH_THUMB, "Thumb", {2, 3}},
  {TCA_CH_INDEX, "Index", {4, 5}},
  {TCA_CH_MIDDLE,"Middle",{6, 13}},
  {TCA_CH_RING,  "Ring",  {12, 11}},
  {TCA_CH_PINKY, "Pinky", {10, 7}}
};

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

VectorInt16 aa1, aaReal1, aaWorld1;
VectorInt16 aa2, aaReal2, aaWorld2;
float ax1, ay1, az1;
float ax2, ay2, az2;

DynamicJsonDocument DataPacket(4096);


// ── Arduino setup/loop ─────────────────────────────────────

void setup() {
  Serial.begin(115200);

  // BLE first (optional, but nice to see logs while sensors init)
  // initBleService();

  initWifi();  

  JsonObject fingerData = DataPacket.createNestedObject("Data");
  buildFingerData(fingerData);

  Wire.begin(I2C_BUS_SDA, I2C_BUS_SCL);
  Wire.setClock(TCA_FREQ);
  TCA.begin(Wire);

  analogReadResolution(12);

  DataPacket["Hand"] = "Left";
  DataPacket["Time"] = nullptr;


  for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
    initFingerChannel(HandChannels[i]);
  }
}

void loop() {
  // Serial.println("working");
  int start = millis();

  for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
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

      IMU_MID.dmpGetAccel(&aa1, fifoBuffer1);
      IMU_MID.dmpGetLinearAccel(&aaReal1, &aa1, &gravity1);
      IMU_MID.dmpGetLinearAccelInWorld(&aaWorld1, &aaReal1, &q1);

      ax1 = aaWorld1.x / 16384.0;
      ay1 = aaWorld1.y / 16384.0;
      az1 = aaWorld1.z / 16384.0;
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

      IMU_PROX.dmpGetAccel(&aa2, fifoBuffer2);
      IMU_PROX.dmpGetLinearAccel(&aaReal2, &aa2, &gravity2);
      IMU_PROX.dmpGetLinearAccelInWorld(&aaWorld2, &aaReal2, &q2);

      ax2 = aaWorld2.x / 16384.0;
      ay2 = aaWorld2.y / 16384.0;
      az2 = aaWorld2.z / 16384.0;
    }

    int MCP_flex = (HandChannels[i].label == "Palm") ? 0 : analogRead(HandChannels[i].adc_channel[0]);
    int PIP_flex = (HandChannels[i].label == "Palm") ? 0 : analogRead(HandChannels[i].adc_channel[1]);

    DataPacket["Time"] = millis();

    DataPacket["Data"][HandChannels[i].label]["flex_mcp"]   = int(MCP_flex);
    DataPacket["Data"][HandChannels[i].label]["flex_pip"]   = int(PIP_flex);

    DataPacket["Data"][HandChannels[i].label]["yaw_prox"]   = roundf(ypr1[0] * 100.0f) / 100.0f;
    DataPacket["Data"][HandChannels[i].label]["pitch_prox"] = roundf(ypr1[1] * 100.0f) / 100.0f;
    DataPacket["Data"][HandChannels[i].label]["roll_prox"]  = roundf(ypr1[2] * 100.0f) / 100.0f;
    DataPacket["Data"][HandChannels[i].label]["ax_prox"]    = roundf(ax1 * 100.0f) / 100.0f;
    DataPacket["Data"][HandChannels[i].label]["ay_prox"]    = roundf(ay1 * 100.0f) / 100.0f;
    DataPacket["Data"][HandChannels[i].label]["az_prox"]    = roundf(az1 * 100.0f) / 100.0f;

    DataPacket["Data"][HandChannels[i].label]["yaw_mid"]    = roundf(ypr2[0] * 100.0f) / 100.0f;
    DataPacket["Data"][HandChannels[i].label]["pitch_mid"]  = roundf(ypr2[1] * 100.0f) / 100.0f;
    DataPacket["Data"][HandChannels[i].label]["roll_mid"]   = roundf(ypr2[2] * 100.0f) / 100.0f;
    DataPacket["Data"][HandChannels[i].label]["ax_mid"]     = roundf(ax2 * 100.0f) / 100.0f;
    DataPacket["Data"][HandChannels[i].label]["ay_mid"]     = roundf(ay2 * 100.0f) / 100.0f;
    DataPacket["Data"][HandChannels[i].label]["az_mid"]     = roundf(az2 * 100.0f) / 100.0f;
  }

  // serializeJsonPretty(DataPacket, Serial);
  // Serial.println();
  // serializeJson(DataPacket, Serial);
  // Serial.println();  // optional newline

  // if (deviceConnected) {
  //   sendJsonOverBle();   // push each frame; or remove if you want "GET" pull only
  // }
  sendJsonOverTcp(DataPacket);

  int end = millis();

  Serial.println("Delay" + String(end - start));

  delay(50);
}

