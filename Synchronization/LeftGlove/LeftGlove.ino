#include <ArduinoJson.h>
#include "Globals.h"
#include "CreateJson.h"
#include "Net.h"
#include "IMU_setup.h"



// ── I2C & TCA config ────────────────────────────────────────
#define I2C_BUS_SDA 8
#define I2C_BUS_SCL 9
#define TCA_FREQ    400000

#define TCA_ADDR    0x71

#define TCA_CH_PALM   1
#define TCA_CH_THUMB  6
#define TCA_CH_INDEX  5
#define TCA_CH_MIDDLE 4
#define TCA_CH_RING   3
#define TCA_CH_PINKY  2
#define TCA_CH_WRIST  7


MPU6050   IMU_MID(0x68);   // AD0 LOW
MPU6050   IMU_PROX(0x69);  // AD0 HIGH
TCA9548A  TCA(TCA_ADDR);

// DynamicJsonDocument doc(4096);

FingerChannel HandChannels[7] = {
  {TCA_CH_PALM,  "Palm",  {-1, -1}, false, false},      // Use PROX IMU
  {TCA_CH_THUMB, "Thumb", {2, 3}, false, false},
  {TCA_CH_INDEX, "Index", {4, 5}, false, false},
  {TCA_CH_MIDDLE,"Middle",{6, 13}, false, false},
  {TCA_CH_RING,  "Ring",  {12, 11}, false, false},
  {TCA_CH_PINKY, "Pinky", {10, 7}, false, false},
  {TCA_CH_WRIST, "Wrist", {-1, -1}, false, false}       // Use PROX IMU
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
// DynamicJsonDocument DataPacket(4096);


volatile bool gloveInitialised = false;
const char* HAND_NAME = "LeftGlove";   // or "LeftGlove"



// ── FIFO helpers ─────────────────────────────────────────────

static bool readDmpMid() {
  if (!dmpReady1 || packetSize1 == 0) return false;
  uint16_t fc = IMU_MID.getFIFOCount();
  if (fc >= 1024) { IMU_MID.resetFIFO(); return false; }
  if (fc < packetSize1) return false;
  while (fc >= packetSize1) {
    IMU_MID.getFIFOBytes(fifoBuffer1, packetSize1);
    fc -= packetSize1;
  }
  IMU_MID.dmpGetQuaternion(&q1, fifoBuffer1);
  IMU_MID.dmpGetGravity(&gravity1, &q1);
  IMU_MID.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
  IMU_MID.dmpGetAccel(&aa1, fifoBuffer1);
  IMU_MID.dmpGetLinearAccel(&aaReal1, &aa1, &gravity1);
  IMU_MID.dmpGetLinearAccelInWorld(&aaWorld1, &aaReal1, &q1);
  ax1 = aaWorld1.x / 16384.0f;
  ay1 = aaWorld1.y / 16384.0f;
  az1 = aaWorld1.z / 16384.0f;
  return true;
}

static bool readDmpProx() {
  if (!dmpReady2 || packetSize2 == 0) return false;
  uint16_t fc = IMU_PROX.getFIFOCount();
  if (fc >= 1024) { IMU_PROX.resetFIFO(); return false; }
  if (fc < packetSize2) return false;
  while (fc >= packetSize2) {
    IMU_PROX.getFIFOBytes(fifoBuffer2, packetSize2);
    fc -= packetSize2;
  }
  IMU_PROX.dmpGetQuaternion(&q2, fifoBuffer2);
  IMU_PROX.dmpGetGravity(&gravity2, &q2);
  IMU_PROX.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
  IMU_PROX.dmpGetAccel(&aa2, fifoBuffer2);
  IMU_PROX.dmpGetLinearAccel(&aaReal2, &aa2, &gravity2);
  IMU_PROX.dmpGetLinearAccelInWorld(&aaWorld2, &aaReal2, &q2);
  ax2 = aaWorld2.x / 16384.0f;
  ay2 = aaWorld2.y / 16384.0f;
  az2 = aaWorld2.z / 16384.0f;
  return true;
}

void handleRequestData(uint32_t requestId, const char* requestTs)
{
  if (!gloveInitialised) return;

  DynamicJsonDocument doc(4096);
  doc["Hand"] = HAND_NAME;
  doc["request_id"] = requestId;
  doc["request_ts"] = requestTs;
  doc["glove_time_ms"] = millis();

  JsonObject fingerData = doc.createNestedObject("Data");


  for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) 
  {
    tcaSelectChannel(HandChannels[i].tca_channel);

    bool gotMid  = HandChannels[i].IMU_MID_EN  ? readDmpMid()  : false;
    bool gotProx = HandChannels[i].IMU_PROX_EN ? readDmpProx() : false;

    // const char* label = HandChannels[i].label;
    String label = HandChannels[i].label;

    int MCP_flex = (HandChannels[i].adc_channel[0] != -1) ? analogRead(HandChannels[i].adc_channel[0]) : -1;
    int PIP_flex = (HandChannels[i].adc_channel[1] != -1) ? analogRead(HandChannels[i].adc_channel[1]) : -1;

    JsonObject finger = fingerData.createNestedObject(label);

  
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
  serializeJson(doc, Serial);
  Serial.println();

  sendJsonOverTcp(doc);
}

void handleInit() {
  // run your IMU setup here
  // e.g. initFingerChannel(...) for all channels you need
  Serial.println("Called handleInit");

  Wire.begin(I2C_BUS_SDA, I2C_BUS_SCL);
  Wire.setClock(TCA_FREQ);

  // Initialise the TCA9548A multiplexer
  TCA.begin(Wire);

  bool status[4] = {false, false, false, false};

  for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++)
  {
    initFingerChannel(HandChannels[i], status);
  } 

  gloveInitialised = true;
  sendReadyMessage(HAND_NAME);
  digitalWrite(1, HIGH);  // for LED
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(1, OUTPUT);
  

  initWifi();


  analogReadResolution(12);

  gloveInitialised = false;
  sendReadyMessage(HAND_NAME);


}

void loop() {
  // put your main code here, to run repeatedly:


  pollTcpCommands(handleInit, handleRequestData);
  // if (!gloveInitialised)
  // {
  //   Serial.println("glove not initialised");
  //   continue;
  // }

  // int start = millis();

  // int MCP_flex = ((HandChannels[i].label == "Palm") || (HandChannels[i].label == "Wrist")) ? -1 : analogRead(HandChannels[i].adc_channel[0]);
  // int PIP_flex = ((HandChannels[i].label == "Palm") || (HandChannels[i].label == "Wrist")) ? -1 : analogRead(HandChannels[i].adc_channel[1]);

  // DataPacket["Time"] = millis();

  // DataPacket["Data"][HandChannels[i].label]["flex_mcp"]   = int(MCP_flex);
  // DataPacket["Data"][HandChannels[i].label]["flex_pip"]   = int(PIP_flex);

  // sendJsonOverTcp(DataPacket);

  // int end = millis();

  // Serial.println("Delay" + String(end - start));

  // delay(50);

}


