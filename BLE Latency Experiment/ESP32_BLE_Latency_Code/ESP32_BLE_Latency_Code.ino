#include <Arduino.h>
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

// Handle data from PC
class MyRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String rxValue = pCharacteristic->getValue();  // Arduino String
    rxValue.trim();

    Serial.print("Received over BLE: ");
    Serial.println(rxValue);

    if (rxValue == "GET") {
      static uint32_t counter = 0;

      // Prepare a quick response (you can put real sensor data here)
      char buffer[64];
      snprintf(buffer, sizeof(buffer),
               "DATA,count=%lu,value=%lu\r\n",
               (unsigned long)counter,
               (unsigned long)(millis() & 0xFFFFFFFF));

      counter++;

      // Send back via notify
      pTxCharacteristic->setValue((uint8_t *)buffer, strlen(buffer));
      pTxCharacteristic->notify();

      Serial.print("Sent over BLE: ");
      Serial.print(buffer);
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting BLE UART for latency test...");

  BLEDevice::init(BLE_DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(UART_SERVICE_UUID);

  // TX characteristic (notify)
  pTxCharacteristic = pService->createCharacteristic(
      UART_CHAR_TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX characteristic (write)
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
      UART_CHAR_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyRxCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(UART_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("BLE UART is advertising, ready to connect.");
}

void loop() {
  // No periodic send; everything is request/response from PC
  delay(10);
}
