#pragma once

#include <Arduino.h>
#include "Globals.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define BLE_DEVICE_NAME "ESP32S3-Zero-BLE"

// Nordic UART Service (NUS) UUIDs
extern BLEUUID UART_SERVICE_UUID;
extern BLEUUID UART_CHAR_TX_UUID; // Notify (ESP32 -> PC)
extern BLEUUID UART_CHAR_RX_UUID; // Write  (PC -> ESP32)

extern BLEServer *pServer;
extern BLECharacteristic *pTxCharacteristi;
extern bool deviceConnected;

// Functions
void initBleService();

void sendJsonOverBle();