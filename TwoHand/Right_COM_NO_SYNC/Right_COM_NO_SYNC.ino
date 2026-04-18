#include <Wire.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#include "Globals.h"
#include "IMU_setup.h"
#include "CreateJson.h"
#include "Net.h"

// ── I2C & TCA config ────────────────────────────────────────
#define I2C_BUS_SDA  8
#define I2C_BUS_SCL  9
#define TCA_FREQ     400000
#define TCA_ADDR     0x71

#define TCA_CH_PALM   1
#define TCA_CH_THUMB  6
#define TCA_CH_INDEX  5
#define TCA_CH_MIDDLE 4
#define TCA_CH_RING   3
#define TCA_CH_PINKY  2
#define TCA_CH_WRIST  7

// ── Definitions of globals declared in Globals.h ────────────
TCA9548A  TCA(TCA_ADDR);
MPU6050   IMU_MID(0x69);
MPU6050   IMU_PROX(0x68);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

FingerChannel HandChannels[6] = {
  {TCA_CH_PALM,   "Palm",   {-1, -1}, false, false},
  {TCA_CH_THUMB,  "Thumb",  {2,  3},  false, false},
  {TCA_CH_INDEX,  "Index",  {4,  5},  false, false},
  {TCA_CH_MIDDLE, "Middle", {6,  13}, false, false},
  {TCA_CH_RING,   "Ring",   {12, 11}, false, false},
  {TCA_CH_PINKY,  "Pinky",  {10, 7},  false, false}
};

bool     dmpReady1   = false;
uint8_t  devStatus1  = 1;
uint16_t packetSize1 = 0;
uint8_t  fifoBuffer1[64];

bool     dmpReady2   = false;
uint8_t  devStatus2  = 1;
uint16_t packetSize2 = 0;
uint8_t  fifoBuffer2[64];

Quaternion  q1, q2;
VectorFloat gravity1, gravity2;
float       ypr1[3] = {0, 0, 0};
float       ypr2[3] = {0, 0, 0};
VectorInt16 aa1, aaReal1, aaWorld1;
VectorInt16 aa2, aaReal2, aaWorld2;
float ax1 = 0, ay1 = 0, az1 = 0;
float ax2 = 0, ay2 = 0, az2 = 0;

DynamicJsonDocument DataPacket(4096);

volatile bool gloveInitialised = false;
const char* HAND_NAME = "RightGlove";   // or "LeftGlove"

// ── FIFO helpers ─────────────────────────────────────────────

// static bool readDmpMid() {
//   if (!dmpReady1 || packetSize1 == 0) return false;
//   uint16_t fc = IMU_MID.getFIFOCount();
//   if (fc >= 1024) { IMU_MID.resetFIFO(); return false; }
//   if (fc < packetSize1) return false;
//   while (fc >= packetSize1) {
//     IMU_MID.getFIFOBytes(fifoBuffer1, packetSize1);
//     fc -= packetSize1;
//   }
//   IMU_MID.dmpGetQuaternion(&q1, fifoBuffer1);
//   IMU_MID.dmpGetGravity(&gravity1, &q1);
//   IMU_MID.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
//   IMU_MID.dmpGetAccel(&aa1, fifoBuffer1);
//   IMU_MID.dmpGetLinearAccel(&aaReal1, &aa1, &gravity1);
//   IMU_MID.dmpGetLinearAccelInWorld(&aaWorld1, &aaReal1, &q1);
//   ax1 = aaWorld1.x / 16384.0f;
//   ay1 = aaWorld1.y / 16384.0f;
//   az1 = aaWorld1.z / 16384.0f;
//   return true;
// }

// static bool readDmpProx() {
//   if (!dmpReady2 || packetSize2 == 0) return false;
//   uint16_t fc = IMU_PROX.getFIFOCount();
//   if (fc >= 1024) { IMU_PROX.resetFIFO(); return false; }
//   if (fc < packetSize2) return false;
//   while (fc >= packetSize2) {
//     IMU_PROX.getFIFOBytes(fifoBuffer2, packetSize2);
//     fc -= packetSize2;
//   }
//   IMU_PROX.dmpGetQuaternion(&q2, fifoBuffer2);
//   IMU_PROX.dmpGetGravity(&gravity2, &q2);
//   IMU_PROX.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
//   IMU_PROX.dmpGetAccel(&aa2, fifoBuffer2);
//   IMU_PROX.dmpGetLinearAccel(&aaReal2, &aa2, &gravity2);
//   IMU_PROX.dmpGetLinearAccelInWorld(&aaWorld2, &aaReal2, &q2);
//   ax2 = aaWorld2.x / 16384.0f;
//   ay2 = aaWorld2.y / 16384.0f;
//   az2 = aaWorld2.z / 16384.0f;
//   return true;
// }

// static bool readDmpMid() {
//   if (!dmpReady1 || packetSize1 == 0) {
//     return false;
//   }

//   // Always start by clearing any backlog / partial data
//   IMU_MID.resetFIFO();

//   // Wait for a fresh complete packet
//   uint32_t start = millis();
//   while (IMU_MID.getFIFOCount() < packetSize1) {
//     // Simple timeout so we don't hang forever if sensor is dead
//     if (millis() - start > 10) {  // 10 ms timeout; tune as needed
//       return false;
//     }
//   }

//   // We have at least one full packet, read exactly one
//   IMU_MID.getFIFOBytes(fifoBuffer1, packetSize1);

//   // Decode DMP data from this packet
//   IMU_MID.dmpGetQuaternion(&q1, fifoBuffer1);
//   IMU_MID.dmpGetGravity(&gravity1, &q1);
//   IMU_MID.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
//   IMU_MID.dmpGetAccel(&aa1, fifoBuffer1);
//   IMU_MID.dmpGetLinearAccel(&aaReal1, &aa1, &gravity1);
//   IMU_MID.dmpGetLinearAccelInWorld(&aaWorld1, &aaReal1, &q1);

//   constexpr float accelScale = 1.0f / 16384.0f;
//   ax1 = aaWorld1.x * accelScale;
//   ay1 = aaWorld1.y * accelScale;
//   az1 = aaWorld1.z * accelScale;

//   return true;
// }

// static bool readDmpProx() {
//   if (!dmpReady2 || packetSize2 == 0) {
//     return false;
//   }

//   // Always start by clearing any backlog / partial data
//   IMU_PROX.resetFIFO();

//   // Wait for a fresh complete packet
//   uint32_t start = millis();
//   while (IMU_PROX.getFIFOCount() < packetSize2) {
//     // Simple timeout so we don't hang forever if sensor is dead
//     if (millis() - start > 10) {  // 10 ms timeout; tune as needed
//       return false;
//     }
//   }

//   // We have at least one full packet, read exactly one
//   IMU_PROX.getFIFOBytes(fifoBuffer2, packetSize2);

//   // Decode DMP data from this packet
//   IMU_PROX.dmpGetQuaternion(&q2, fifoBuffer2);
//   IMU_PROX.dmpGetGravity(&gravity2, &q2);
//   IMU_PROX.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
//   IMU_PROX.dmpGetAccel(&aa2, fifoBuffer2);
//   IMU_PROX.dmpGetLinearAccel(&aaReal2, &aa2, &gravity2);
//   IMU_PROX.dmpGetLinearAccelInWorld(&aaWorld2, &aaReal2, &q2);

//   constexpr float accelScale = 1.0f / 16384.0f;
//   ax2 = aaWorld2.x * accelScale;
//   ay2 = aaWorld2.y * accelScale;
//   az2 = aaWorld2.z * accelScale;

//   return true;
// }

static bool readDmpMid() {
  if (!dmpReady1 || packetSize1 == 0) {
    return false;
  }

  uint16_t fc = IMU_MID.getFIFOCount();

  if (fc >= 1024) {
    IMU_MID.resetFIFO();
    return false;
  }

  if (fc < packetSize1) {
    return false;
  }

  // Read exactly one packet (the oldest complete packet currently queued)
  IMU_MID.getFIFOBytes(fifoBuffer1, packetSize1);

  // Decode DMP data from this packet
  IMU_MID.dmpGetQuaternion(&q1, fifoBuffer1);
  IMU_MID.dmpGetGravity(&gravity1, &q1);
  IMU_MID.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
  IMU_MID.dmpGetAccel(&aa1, fifoBuffer1);
  IMU_MID.dmpGetLinearAccel(&aaReal1, &aa1, &gravity1);
  IMU_MID.dmpGetLinearAccelInWorld(&aaWorld1, &aaReal1, &q1);

  constexpr float accelScale = 1.0f / 16384.0f;
  ax1 = aaWorld1.x * accelScale;
  ay1 = aaWorld1.y * accelScale;
  az1 = aaWorld1.z * accelScale;

  // Flush any remaining backlog so next loop stays fast
  if (fc > packetSize1) {
    IMU_MID.resetFIFO();
  }

  return true;
}

static bool readDmpProx() {
  if (!dmpReady2 || packetSize2 == 0) {
    return false;
  }

  uint16_t fc = IMU_PROX.getFIFOCount();

  if (fc >= 1024) {
    IMU_PROX.resetFIFO();
    return false;
  }

  if (fc < packetSize2) {
    return false;
  }

  // Read exactly one packet (the oldest complete packet currently queued)
  IMU_PROX.getFIFOBytes(fifoBuffer2, packetSize2);

  // Decode DMP data from this packet
  IMU_PROX.dmpGetQuaternion(&q2, fifoBuffer2);
  IMU_PROX.dmpGetGravity(&gravity2, &q2);
  IMU_PROX.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
  IMU_PROX.dmpGetAccel(&aa2, fifoBuffer2);
  IMU_PROX.dmpGetLinearAccel(&aaReal2, &aa2, &gravity2);
  IMU_PROX.dmpGetLinearAccelInWorld(&aaWorld2, &aaReal2, &q2);

  constexpr float accelScale = 1.0f / 16384.0f;
  ax2 = aaWorld2.x * accelScale;
  ay2 = aaWorld2.y * accelScale;
  az2 = aaWorld2.z * accelScale;

  // Flush any remaining backlog so next loop stays fast
  if (fc > packetSize2) {
    IMU_PROX.resetFIFO();
  }

  return true;
}


// ── setup ────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("\nBooting...");


  Wire.begin(I2C_BUS_SDA, I2C_BUS_SCL);
  Wire.setClock(TCA_FREQ);
  Wire.setTimeOut(10); // 10ms max per transaction — fail fast
  TCA.begin(Wire);
  analogReadResolution(12);

  tcaSelectChannel(TCA_CH_WRIST);

  delay(10);

  if (!bno.begin()) {
    Serial.println("BNO055 not detected! Check wiring or TCA channel.");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 ready on TCA Channel 1.");

  bool status[4] = {false, false, false, false};
  for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
    initFingerChannel(HandChannels[i], status);
    Serial.println("---");
    Serial.println(HandChannels[i].label);
    Serial.println(String("IMU_MID      : ") + (status[0] ? "OK" : "FAIL"));
    Serial.println(String("IMU_MID_DMP  : ") + (status[1] ? "OK" : "FAIL"));
    Serial.println(String("IMU_PROX     : ") + (status[2] ? "OK" : "FAIL"));
    Serial.println(String("IMU_PROX_DMP : ") + (status[3] ? "OK" : "FAIL"));
    status[0] = status[1] = status[2] = status[3] = false;
  }

  initWifi();
  gloveInitialised = true;
  digitalWrite(1, HIGH);  // for LEDsendReadyMessage(HAND_NAME);

}



// ── loop ─────────────────────────────────────────────────────

void loop() {

  if (!gloveInitialised) return;

  DynamicJsonDocument doc(4096);
  doc["Hand"]         = HAND_NAME;
  doc["glove_time_ms"] = millis();

  JsonObject fingerData = doc.createNestedObject("Data");

  for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
    FingerChannel &fc = HandChannels[i];
    tcaSelectChannel(fc.tca_channel);

    bool gotMid  = fc.IMU_MID_EN  ? readDmpMid()  : false;
    bool gotProx = fc.IMU_PROX_EN ? readDmpProx() : false;

    int MCP_flex = (fc.adc_channel[0] != -1) ? analogRead(fc.adc_channel[0]) : -1;
    int PIP_flex = (fc.adc_channel[1] != -1) ? analogRead(fc.adc_channel[1]) : -1;

    // ↓ createNestedObject on fingerData, not a lookup on doc["Data"]
    JsonObject finger = fingerData.createNestedObject(fc.label);

    finger["flex_mcp"] = MCP_flex;
    finger["flex_pip"] = PIP_flex;

    finger["yaw_mid"]   = gotMid ? roundf(ypr1[0] * 100.0f) / 100.0f : 0.0f;
    finger["pitch_mid"] = gotMid ? roundf(ypr1[1] * 100.0f) / 100.0f : 0.0f;
    finger["roll_mid"]  = gotMid ? roundf(ypr1[2] * 100.0f) / 100.0f : 0.0f;
    finger["ax_mid"]    = gotMid ? roundf(ax1 * 100.0f) / 100.0f     : 0.0f;
    finger["ay_mid"]    = gotMid ? roundf(ay1 * 100.0f) / 100.0f     : 0.0f;
    finger["az_mid"]    = gotMid ? roundf(az1 * 100.0f) / 100.0f     : 0.0f;

    finger["yaw_prox"]   = gotProx ? roundf(ypr2[0] * 100.0f) / 100.0f : 0.0f;
    finger["pitch_prox"] = gotProx ? roundf(ypr2[1] * 100.0f) / 100.0f : 0.0f;
    finger["roll_prox"]  = gotProx ? roundf(ypr2[2] * 100.0f) / 100.0f : 0.0f;
    finger["ax_prox"]    = gotProx ? roundf(ax2 * 100.0f) / 100.0f     : 0.0f;
    finger["ay_prox"]    = gotProx ? roundf(ay2 * 100.0f) / 100.0f     : 0.0f;
    finger["az_prox"]    = gotProx ? roundf(az2 * 100.0f) / 100.0f     : 0.0f;
  }

   tcaSelectChannel(TCA_CH_WRIST);   // <-- add this

  JsonObject finger = fingerData.createNestedObject("Wrist");

  // --- Euler Angles ---
  sensors_event_t orientEvent;
  bno.getEvent(&orientEvent);

  // --- Acceleration ---
  sensors_event_t accelEvent;
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);


  finger["ax"]       = roundf(accelEvent.acceleration.x * 100.0f) / 100.0f;
  finger["ay"]       = roundf(accelEvent.acceleration.y * 100.0f) / 100.0f;
  finger["az"]       = roundf(accelEvent.acceleration.z * 100.0f) / 100.0f;

  finger["heading"]  = roundf(orientEvent.orientation.x * 100.0f) / 100.0f;
  finger["pitch"]    = roundf(orientEvent.orientation.y * 100.0f) / 100.0f;
  finger["roll"]     = roundf(orientEvent.orientation.z * 100.0f) / 100.0f;


  doc["Time"] = millis();   // moved outside the loop — only needs to be set once

  serializeJson(doc, Serial);
  Serial.println();

  sendJsonOverTcp(doc);

  delay(5);
}

// void handleInit() {

//   bool status[4] = {false, false, false, false};
//   for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
//     initFingerChannel(HandChannels[i], status);
//     Serial.println("---");
//     Serial.println(HandChannels[i].label);
//     Serial.println(String("IMU_MID      : ") + (status[0] ? "OK" : "FAIL"));
//     Serial.println(String("IMU_MID_DMP  : ") + (status[1] ? "OK" : "FAIL"));
//     Serial.println(String("IMU_PROX     : ") + (status[2] ? "OK" : "FAIL"));
//     Serial.println(String("IMU_PROX_DMP : ") + (status[3] ? "OK" : "FAIL"));
//     status[0] = status[1] = status[2] = status[3] = false;
//   }

//   gloveInitialised = true;
//   sendReadyMessage(HAND_NAME);
//   digitalWrite(1, HIGH);  // for LEDsendReadyMessage(HAND_NAME);
// }

// void handleRequestData(uint32_t requestId, const char* requestTs)
// {
//   if (!gloveInitialised) return;

//   DynamicJsonDocument doc(4096);
//   doc["Hand"]         = HAND_NAME;
//   doc["request_id"]   = requestId;
//   doc["request_ts"]   = requestTs;
//   doc["glove_time_ms"] = millis();

//   JsonObject fingerData = doc.createNestedObject("Data");

//   for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
//     FingerChannel &fc = HandChannels[i];
//     tcaSelectChannel(fc.tca_channel);

//     bool gotMid  = fc.IMU_MID_EN  ? readDmpMid()  : false;
//     bool gotProx = fc.IMU_PROX_EN ? readDmpProx() : false;

//     int MCP_flex = (fc.adc_channel[0] != -1) ? analogRead(fc.adc_channel[0]) : -1;
//     int PIP_flex = (fc.adc_channel[1] != -1) ? analogRead(fc.adc_channel[1]) : -1;

//     // ↓ createNestedObject on fingerData, not a lookup on doc["Data"]
//     JsonObject finger = fingerData.createNestedObject(fc.label);

//     finger["flex_mcp"] = MCP_flex;
//     finger["flex_pip"] = PIP_flex;

//     finger["yaw_mid"]   = gotMid ? roundf(ypr1[0] * 100.0f) / 100.0f : 0.0f;
//     finger["pitch_mid"] = gotMid ? roundf(ypr1[1] * 100.0f) / 100.0f : 0.0f;
//     finger["roll_mid"]  = gotMid ? roundf(ypr1[2] * 100.0f) / 100.0f : 0.0f;
//     finger["ax_mid"]    = gotMid ? roundf(ax1 * 100.0f) / 100.0f     : 0.0f;
//     finger["ay_mid"]    = gotMid ? roundf(ay1 * 100.0f) / 100.0f     : 0.0f;
//     finger["az_mid"]    = gotMid ? roundf(az1 * 100.0f) / 100.0f     : 0.0f;

//     finger["yaw_prox"]   = gotProx ? roundf(ypr2[0] * 100.0f) / 100.0f : 0.0f;
//     finger["pitch_prox"] = gotProx ? roundf(ypr2[1] * 100.0f) / 100.0f : 0.0f;
//     finger["roll_prox"]  = gotProx ? roundf(ypr2[2] * 100.0f) / 100.0f : 0.0f;
//     finger["ax_prox"]    = gotProx ? roundf(ax2 * 100.0f) / 100.0f     : 0.0f;
//     finger["ay_prox"]    = gotProx ? roundf(ay2 * 100.0f) / 100.0f     : 0.0f;
//     finger["az_prox"]    = gotProx ? roundf(az2 * 100.0f) / 100.0f     : 0.0f;
//   }

//   doc["Time"] = millis();   // moved outside the loop — only needs to be set once

//   serializeJson(doc, Serial);
//   Serial.println();

//   sendJsonOverTcp(doc);
// }