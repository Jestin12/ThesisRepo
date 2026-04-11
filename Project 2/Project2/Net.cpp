#include "Net.h"

// ===== USER SETTINGS =====
static const char* WIFI_SSID     = "Belong96E660";
static const char* WIFI_PASSWORD = "u7255uutshe3gzaq";

// PC server config (change to your PC's LAN IP)
static const char* TCP_HOST      = "192.168.1.57";  // your computer IP
static const uint16_t TCP_PORT   = 5000;
// ==========================

static WiFiClient tcpClient;
static unsigned long lastConnectAttempt = 0;

static void ensureTcpConnected() {
  if (tcpClient.connected()) return;

  unsigned long now = millis();
  if (now - lastConnectAttempt < 2000) return;  // retry every 2s

  lastConnectAttempt = now;
  Serial.print("Connecting TCP to ");
  Serial.print(TCP_HOST);
  Serial.print(":");
  Serial.println(TCP_PORT);

  if (tcpClient.connect(TCP_HOST, TCP_PORT)) {
    Serial.println("TCP connected");
  } else {
    Serial.println("TCP connect failed");
  }
}

void initWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Wi-Fi connected, IP: ");
  Serial.println(WiFi.localIP());

  // First TCP attempt (subsequent handled in ensureTcpConnected)
  ensureTcpConnected();
}

void sendJsonOverTcp(const DynamicJsonDocument& doc) {
  ensureTcpConnected();
  if (!tcpClient.connected()) return;

  char jsonBuf[4096];

  // leave room for '\n' + '\0'
  size_t len = serializeJson(doc, jsonBuf, sizeof(jsonBuf) - 2);
  if (len == 0 || len >= sizeof(jsonBuf) - 2) {
    Serial.println("TCP: JSON too large or error serializing");
    return;
  }

  jsonBuf[len] = '\n';
  jsonBuf[len + 1] = '\0';
  len += 1;

  size_t written = tcpClient.write((uint8_t*)jsonBuf, len);
  if (written != len) {
    Serial.println("TCP: short write");
  }
}