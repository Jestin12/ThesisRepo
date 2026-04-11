#include <ArduinoJson.h>
#include "Globals.h"
#include "CreateJson.h"
#include "Net.h"



// ── I2C & TCA config ────────────────────────────────────────
#define I2C_BUS_SDA 8
#define I2C_BUS_SCL 9
#define TCA_FREQ    100000

#define TCA_ADDR    0x71

#define TCA_CH_PALM   1
#define TCA_CH_THUMB  6
#define TCA_CH_INDEX  5
#define TCA_CH_MIDDLE 4
#define TCA_CH_RING   3
#define TCA_CH_PINKY  2
#define TCA_CH_WRIST  7

DynamicJsonDocument DataPacket(4096);

FingerChannel HandChannels[7] = {
  {TCA_CH_PALM,  "Palm",  {}, false, false},      // Use PROX IMU
  {TCA_CH_THUMB, "Thumb", {2, 3}, false, false},
  {TCA_CH_INDEX, "Index", {4, 5}, false, false},
  {TCA_CH_MIDDLE,"Middle",{6, 13}, false, false},
  {TCA_CH_RING,  "Ring",  {12, 11}, false, false},
  {TCA_CH_PINKY, "Pinky", {10, 7}, false, false},
  {TCA_CH_WRIST, "Wrist", {}, false, false}       // Use PROX IMU
};

volatile bool gloveInitialised = false;
const char* HAND_NAME = "RightGlove";   // or "LeftGlove"


void handleRequestData();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  initWifi();

  JsonObject fingerData = DataPacket.createNestedObject("Data");
  buildFingerData(fingerData);

  analogReadResolution(12);

  DataPacket["Hand"] = "Right";
  DataPacket["Time"] = nullptr;

  gloveInitialised = true;
  sendReadyMessage(HAND_NAME);

}

void loop() {
  // put your main code here, to run repeatedly:

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


void handleRequestData() {
  if (!gloveInitialised) return;

  DynamicJsonDocument doc(4096);
  doc["Hand"] = HAND_NAME;
  doc["Time"] = millis();

  // populate doc["Data"] = ...
  // then:
  sendJsonOverTcp(doc);
}
