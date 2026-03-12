#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define BLE_DEVICE_NAME "ESP32S3-Zero-BLE"

// Nordic UART Service (NUS) UUIDs
static BLEUUID UART_SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID UART_CHAR_TX_UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // Notify (ESP32 -> PC)
static BLEUUID UART_CHAR_RX_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // Write  (PC -> ESP32)

BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;

// ------------ I2C / MUX / MPU CONFIG ------------

// Mux 1 I2C on GPIO18/39, Mux 2 on GPIO41/45
#define MUX1_SDA   18
#define MUX1_SCL   39
#define MUX2_SDA   41
#define MUX2_SCL   45

#define TCA9548A_ADDR 0x70    // both muxes same address (A0–A2 wired same)
#define MPU6050_ADDR  0x68    // AD0 = GND

TwoWire I2C1 = TwoWire(0);
TwoWire I2C2 = TwoWire(1);

const uint8_t mux1Channels[] = {3,4,5,6,7};    // mux1: 3–7 used
const uint8_t mux2Channels[] = {2,3,4,5,6,7};  // mux2: 2–7 used

// MPU6050 registers
#define MPU6050_RA_PWR_MGMT_1   0x6B
#define MPU6050_RA_SMPLRT_DIV   0x19
#define MPU6050_RA_CONFIG       0x1A
#define MPU6050_RA_GYRO_CONFIG  0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_ACCEL_XOUT_H 0x3B

struct MPUData {
  bool   valid;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t t;
};

// ------------ BLE SERVER CALLBACKS ------------

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    deviceConnected = true;
    Serial.println("BLE client connected");
  }

  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    Serial.println("BLE client disconnected");
    pServer->getAdvertising()->start();
  }
};

// ------------ I2C / MPU HELPERS ------------

bool muxSelectChannel(TwoWire &bus, uint8_t channel) {
  uint8_t ctrl = (1 << channel);
  bus.beginTransmission(TCA9548A_ADDR);
  bus.write(ctrl);
  return (bus.endTransmission() == 0);
}

bool mpuWriteReg(TwoWire &bus, uint8_t reg, uint8_t val) {
  bus.beginTransmission(MPU6050_ADDR);
  bus.write(reg);
  bus.write(val);
  return (bus.endTransmission() == 0);
}

bool mpuReadBytes(TwoWire &bus, uint8_t reg, uint8_t len, uint8_t *data) {
  bus.beginTransmission(MPU6050_ADDR);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom((int)MPU6050_ADDR, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) data[i] = bus.read();
  return true;
}

bool mpuInitOnChannel(TwoWire &bus, uint8_t channel) {
  if (!muxSelectChannel(bus, channel)) return false;
  delay(2);
  if (!mpuWriteReg(bus, MPU6050_RA_PWR_MGMT_1, 0x01)) return false;
  delay(10);
  if (!mpuWriteReg(bus, MPU6050_RA_SMPLRT_DIV, 9)) return false;   // 100 Hz
  if (!mpuWriteReg(bus, MPU6050_RA_CONFIG, 0x03)) return false;    // DLPF
  if (!mpuWriteReg(bus, MPU6050_RA_GYRO_CONFIG, 0x18)) return false;   // ±2000 dps
  if (!mpuWriteReg(bus, MPU6050_RA_ACCEL_CONFIG, 0x08)) return false;  // ±4g
  return true;
}

bool mpuReadOnChannel(TwoWire &bus, uint8_t channel, MPUData &out) {
  if (!muxSelectChannel(bus, channel)) {
    out.valid = false;
    return false;
  }
  uint8_t buf[14];
  if (!mpuReadBytes(bus, MPU6050_RA_ACCEL_XOUT_H, 14, buf)) {
    out.valid = false;
    return false;
  }
  out.ax = (int16_t)((buf[0] << 8) | buf[1]);
  out.ay = (int16_t)((buf[2] << 8) | buf[3]);
  out.az = (int16_t)((buf[4] << 8) | buf[5]);
  out.t  = (int16_t)((buf[6] << 8) | buf[7]);
  out.gx = (int16_t)((buf[8] << 8) | buf[9]);
  out.gy = (int16_t)((buf[10] << 8) | buf[11]);
  out.gz = (int16_t)((buf[12] << 8) | buf[13]);
  out.valid = true;
  return true;
}

String buildImuJson(uint64_t ts_us,
                    const MPUData mux1Data[], size_t mux1Count,
                    const MPUData mux2Data[], size_t mux2Count) {
  String json = "{";
  json += "\"ts_us\":" + String((unsigned long long)ts_us) + ",";
  json += "\"mux1\":[";
  for (size_t i = 0; i < mux1Count; i++) {
    if (i) json += ",";
    if (!mux1Data[i].valid) { json += "null"; continue; }
    json += "{";
    json += "\"ch\":" + String(mux1Channels[i]) + ",";
    json += "\"ax\":" + String(mux1Data[i].ax) + ",";
    json += "\"ay\":" + String(mux1Data[i].ay) + ",";
    json += "\"az\":" + String(mux1Data[i].az) + ",";
    json += "\"gx\":" + String(mux1Data[i].gx) + ",";
    json += "\"gy\":" + String(mux1Data[i].gy) + ",";
    json += "\"gz\":" + String(mux1Data[i].gz) + ",";
    json += "\"t\":"  + String(mux1Data[i].t);
    json += "}";
  }
  json += "],\"mux2\":[";
  for (size_t i = 0; i < mux2Count; i++) {
    if (i) json += ",";
    if (!mux2Data[i].valid) { json += "null"; continue; }
    json += "{";
    json += "\"ch\":" + String(mux2Channels[i]) + ",";
    json += "\"ax\":" + String(mux2Data[i].ax) + ",";
    json += "\"ay\":" + String(mux2Data[i].ay) + ",";
    json += "\"az\":" + String(mux2Data[i].az) + ",";
    json += "\"gx\":" + String(mux2Data[i].gx) + ",";
    json += "\"gy\":" + String(mux2Data[i].gy) + ",";
    json += "\"gz\":" + String(mux2Data[i].gz) + ",";
    json += "\"t\":"  + String(mux2Data[i].t);
    json += "}";
  }
  json += "]}";
  return json;
}

String pollAllMPUs() {
  const uint64_t start_us = esp_timer_get_time();

  MPUData mux1Data[sizeof(mux1Channels)];
  MPUData mux2Data[sizeof(mux2Channels)];

  for (size_t i = 0; i < sizeof(mux1Channels); i++) {
    if ((esp_timer_get_time() - start_us) > 100000) return String();
    mpuReadOnChannel(I2C1, mux1Channels[i], mux1Data[i]);
  }
  for (size_t i = 0; i < sizeof(mux2Channels); i++) {
    if ((esp_timer_get_time() - start_us) > 100000) return String();
    mpuReadOnChannel(I2C2, mux2Channels[i], mux2Data[i]);
  }

  uint64_t ts = esp_timer_get_time();
  return buildImuJson(ts, mux1Data, sizeof(mux1Channels),
                      mux2Data, sizeof(mux2Channels));
}

// ------------ BLE RX CALLBACK (PC -> ESP32) ------------

class MyRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    if (!pTxCharacteristic) {
      Serial.println("TX characteristic not initialised!");
      return;
    }

    String rxValue = pCharacteristic->getValue();
    rxValue.trim();

    Serial.print("Received over BLE: ");
    Serial.println(rxValue);

    if (rxValue == "GET") {
      String json;
      do {
        json = pollAllMPUs();
      } while (json.length() == 0);

      json += "\n";

      pTxCharacteristic->setValue((uint8_t *)json.c_str(), json.length());
      pTxCharacteristic->notify();

      Serial.print("Sent IMU JSON over BLE: ");
      Serial.print(json);
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting BLE UART with IMU JSON...");

  // 1) Bring up BLE first, with checks
  BLEDevice::init(BLE_DEVICE_NAME);

  pServer = BLEDevice::createServer();
  if (!pServer) {
    Serial.println("Failed to create BLE server!");
    while (true) delay(1000);
  }
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(UART_SERVICE_UUID);
  if (!pService) {
    Serial.println("Failed to create BLE service!");
    while (true) delay(1000);
  }

  pTxCharacteristic = pService->createCharacteristic(
      UART_CHAR_TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  if (!pTxCharacteristic) {
    Serial.println("Failed to create TX characteristic!");
    while (true) delay(1000);
  }
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
      UART_CHAR_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  if (!pRxCharacteristic) {
    Serial.println("Failed to create RX characteristic!");
    while (true) delay(1000);
  }
  pRxCharacteristic->setCallbacks(new MyRxCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  if (!pAdvertising) {
    Serial.println("Failed to get advertising!");
    while (true) delay(1000);
  }
  pAdvertising->addServiceUUID(UART_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("BLE UART is advertising, ready to connect.");

  // 2) Now bring up I2C + MPUs
  I2C1.begin(MUX1_SDA, MUX1_SCL, 400000);
  I2C2.begin(MUX2_SDA, MUX2_SCL, 400000);

  for (uint8_t ch : mux1Channels) {
    mpuInitOnChannel(I2C1, ch);
    delay(2);
  }
  for (uint8_t ch : mux2Channels) {
    mpuInitOnChannel(I2C2, ch);
    delay(2);
  }

  Serial.println("I2C + IMUs initialised.");
}

void loop() {
  delay(10);
}
