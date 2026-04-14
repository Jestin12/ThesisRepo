#include "BLE.h"
#include <ArduinoJson.h>



// UUID definitions (unchanged)
BLEUUID UART_SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLEUUID UART_CHAR_TX_UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
BLEUUID UART_CHAR_RX_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

// BLE globals
BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;

// Track effective payload size (MTU minus ATT header)
uint16_t blePayloadSize = 20;  // default safe value


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    deviceConnected = true;
    Serial.println("BLE client connected");

    // Get negotiated MTU from the connection
    uint16_t mtu = pServer->getConnId() >= 0
                     ? pServer->getPeerMTU(pServer->getConnId())
                     : 23;  // default
    if (mtu < 23) mtu = 23;
    blePayloadSize = (mtu > 23) ? (mtu - 3) : 20;  // 3 bytes ATT header
    Serial.print("Negotiated MTU: ");
    Serial.print(mtu);
    Serial.print(" -> payload ");
    Serial.println(blePayloadSize);
  }

  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    Serial.println("BLE client disconnected");
    pServer->getAdvertising()->start();
  }
};


class MyRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    
    String rxValue = pCharacteristic->getValue();
    rxValue.trim();

    Serial.print("Received over BLE: ");
    Serial.println(rxValue);

    if (rxValue == "GET") {
      sendJsonOverBle();
    }
  }
};

void initBleService() {
  BLEDevice::init(BLE_DEVICE_NAME);

  // Request a larger MTU (peer must also accept)
  BLEDevice::setMTU(512);  // between 23 and 517 [web:40][web:42][web:50]

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(UART_SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    UART_CHAR_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    UART_CHAR_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new MyRxCallbacks());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("BLE JSON service started");
}

void sendJsonOverBle() {
  if (!deviceConnected) return;

  // Adjust size to comfortably hold your JSON (you had 1024 bytes before)
  static char jsonBuf[1024];

  // Serialize without pretty-printing to keep it compact
  size_t len = serializeJson(DataPacket, jsonBuf, sizeof(jsonBuf));

  if (len == 0 || len >= sizeof(jsonBuf)) {
    Serial.println("JSON too large or error serializing");
    return;
  }

  Serial.print("JSON length: ");
  Serial.println(len);

  size_t offset = 0;
  while (offset < len) {
    size_t chunkSize = min((size_t)blePayloadSize, len - offset);
    pTxCharacteristic->setValue((uint8_t *)(jsonBuf + offset), chunkSize);
    pTxCharacteristic->notify();
    offset += chunkSize;

    // Small delay can help some phone stacks cope with rapid notifications
    delay(2);
  }

  Serial.println("JSON sent over BLE (chunked)");
}