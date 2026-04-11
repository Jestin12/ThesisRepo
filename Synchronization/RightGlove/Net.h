#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// Call once from setup()
void initWifi();

// Call periodically from loop() to send the current DataPacket frame
void sendJsonOverTcp(const DynamicJsonDocument& doc);