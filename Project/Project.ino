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
// StaticJsonDocument<1024> DataPacket;

// void tcaSelectChannel(uint8_t ch) {
//   TCA.closeAll();
//   TCA.openChannel(ch);
// }

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
    

  // for (int i = 0; i < int(sizeof(HandChannels)/sizeof(HandChannels[0])); i++)
  // {
  //   tcaSelectChannel(HandChannels[i].tca_channel);

  //   Serial.println("Initialising " + HandChannels[i].label);


  //   IMU_MID.initialize();
  //   IMU_PROX.initialize();

  //   Serial.println(IMU_PROX.testConnection() ? "MPU6050 " + HandChannels[i].label + " Proximal OK" : "MPU6050 " + HandChannels[i].label + " Proximal FAIL");
  //   Serial.println(IMU_MID.testConnection() ? "MPU6050 " + HandChannels[i].label + " Mid OK" : "MPU6050 " + HandChannels[i].label + " Mid FAIL");

  //   Serial.print("WHOAMI1: 0x");
  //   Serial.println(IMU_MID.getDeviceID(), HEX);
  //   Serial.print("WHOAMI2: 0x");
  //   Serial.println(IMU_PROX.getDeviceID(), HEX);

  //   Serial.println(F("Initializing DMP..."));
  //   devStatus1 = IMU_MID.dmpInitialize();
  //   devStatus2 = IMU_PROX.dmpInitialize();  // <-- FIXED: use IMU_PROX here

  //   Serial.print("devStatus1 = ");
  //   Serial.println(devStatus1);
  //   Serial.print("devStatus2 = ");
  //   Serial.println(devStatus2);

  //   if (devStatus1 == 0) {
  //     IMU_MID.CalibrateAccel(6);
  //     IMU_MID.CalibrateGyro(6);
  //     IMU_MID.PrintActiveOffsets();

  //     Serial.println(F("Enabling DMP for proximal phalangeal..."));
  //     IMU_MID.setDMPEnabled(true);

  //     packetSize1 = IMU_MID.dmpGetFIFOPacketSize();
  //     // dmpReady1   = true;
  //   } else {
  //     Serial.print(F("proximal phalangeal DMP init failed (code "));
  //     Serial.print(devStatus1);
  //     Serial.println(F(")"));
  //     dmpReady2   = false;
  //   }

  //   if (devStatus2 == 0) {
  //     IMU_PROX.CalibrateAccel(6);
  //     IMU_PROX.CalibrateGyro(6);
  //     IMU_PROX.PrintActiveOffsets();

  //     Serial.println(F("Enabling DMP for mid phalangeal..."));
  //     IMU_PROX.setDMPEnabled(true);

  //     packetSize2 = IMU_PROX.dmpGetFIFOPacketSize();
  //     // dmpReady2   = true;
  //   } else {
  //     Serial.print(F("mid phalangeal DMP init failed (code "));
  //     Serial.print(devStatus2);
  //     Serial.println(F(")"));
  //     dmpReady2   = false;
  //   }
  // }

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

    DataPacket["Data"][HandChannels[i].label]["flex_mcp"];
    DataPacket["Data"][HandChannels[i].label]["flex_pip"];

    Serial.print(HandChannels[i].label+"\n");
    Serial.print("ypr1[");
    Serial.print(ypr1[0] * 180.0 / M_PI); Serial.print(", ");
    Serial.print(ypr1[1] * 180.0 / M_PI); Serial.print(", ");
    Serial.print(ypr1[2] * 180.0 / M_PI); Serial.println("]");

    Serial.print("ypr2[");
    Serial.print(ypr2[0] * 180.0 / M_PI); Serial.print(", ");
    Serial.print(ypr2[1] * 180.0 / M_PI); Serial.print(", ");
    Serial.print(ypr2[2] * 180.0 / M_PI); Serial.println("]");

    

    delay(20);
  }
    
}