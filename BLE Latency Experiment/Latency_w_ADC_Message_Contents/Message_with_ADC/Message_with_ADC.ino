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

// ADC pins 7–16 (GPIO numbers)
const int adcPins[] = {7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
const size_t NUM_ADC = sizeof(adcPins) / sizeof(adcPins[0]);

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

// Build JSON string with ADC readings
void buildAdcJson(char *buffer, size_t bufSize) {
  // Example JSON:
  // {"adc":[v7,v8,...,v16]}
  // Keep it short (< 200 bytes) to fit in one BLE notification.
  int n = snprintf(buffer, bufSize, "{\"adc\":[");
  for (size_t i = 0; i < NUM_ADC; ++i) {
    int val = analogRead(adcPins[i]);
    n += snprintf(buffer + n, bufSize - n, "%d", val);
    if (i + 1 < NUM_ADC) {
      n += snprintf(buffer + n, bufSize - n, ",");
    }
    if (n >= (int)bufSize - 2) break;  // safety
  }
  snprintf(buffer + n, bufSize - n, "]}");
}

// Handle data from PC
class MyRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String rxValue = pCharacteristic->getValue();
    rxValue.trim();

    Serial.print("Received over BLE: ");
    Serial.println(rxValue);

    if (rxValue == "GET") {
      char jsonBuf[256];
      buildAdcJson(jsonBuf, sizeof(jsonBuf));

      // Append newline for easy line-based parsing on PC
      size_t len = strnlen(jsonBuf, sizeof(jsonBuf) - 2);
      jsonBuf[len] = '\n';
      jsonBuf[len + 1] = '\0';

      pTxCharacteristic->setValue((uint8_t *)jsonBuf, strlen(jsonBuf));
      pTxCharacteristic->notify();

      Serial.print("Sent JSON over BLE: ");
      Serial.print(jsonBuf);
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting BLE UART with ADC JSON...");

  // Configure ADC pins
  for (size_t i = 0; i < NUM_ADC; ++i) {
    pinMode(adcPins[i], INPUT);
  }

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
  // Everything is request/response; nothing periodic here.
  delay(10);
}
