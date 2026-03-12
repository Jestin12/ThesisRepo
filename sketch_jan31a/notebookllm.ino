#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include "BluetoothSerial.h"

// ================= CONFIGURATION =================

// --- Bluetooth Setup ---
BluetoothSerial SerialBT;
const char* deviceName = "ESP32S3_SmartGlove";

// --- I2C Bus 1 (Multiplexer 1) ---
#define I2C1_SDA 1
#define I2C1_SCL 2
TwoWire I2C_BUS_1 = TwoWire(0); // Hardware I2C 0

// --- I2C Bus 2 (Multiplexer 2) ---
#define I2C2_SDA 3
#define I2C2_SCL 4
TwoWire I2C_BUS_2 = TwoWire(1); // Hardware I2C 1

// --- Multiplexer Address ---
#define TCA_ADDR 0x70 

// --- Flex Sensor Pins (ADC) ---
// Ensure these are valid ADC pins on your specific S3 breakout
const int flexPins[4] = {5, 6, 7, 8, 9, 10, 11, 12, 13, 14};

// --- IMU Management ---
#define TOTAL_IMUS 11
Adafruit_MPU6050 mpus[TOTAL_IMUS];

// Structure to map each IMU to a specific Bus and Mux Channel
struct ImuMap {
  TwoWire* wireBus; // Which I2C bus (1 or 2)
  uint8_t muxChannel; // Which channel on the TCA9548A (0-7)
};

// Mapping: First 8 IMUs on Bus 1, Next 3 IMUs on Bus 2
ImuMap imuMapping[TOTAL_IMUS] = {
  {&I2C_BUS_1, 0}, {&I2C_BUS_1, 1}, {&I2C_BUS_1, 2}, {&I2C_BUS_1, 3},
  {&I2C_BUS_1, 4}, {&I2C_BUS_1, 5}, {&I2C_BUS_1, 6}, {&I2C_BUS_1, 7},
  {&I2C_BUS_2, 0}, {&I2C_BUS_2, 1}, {&I2C_BUS_2, 2}
};

// ================= HELPERS =================

// Switch Multiplexer Channel
void tcaSelect(TwoWire *wireBus, uint8_t i) {
  if (i > 7) return;
  wireBus->beginTransmission(TCA_ADDR);
  wireBus->write(1 << i);
  wireBus->endTransmission();
}

void setup() {
  Serial.begin(115200);
  
  // 1. Initialize Bluetooth
  SerialBT.begin(deviceName);
  Serial.println("Bluetooth Started! Ready to pair.");

  // 2. Initialize Dual I2C Buses
  // ESP32-S3 allows mapping I2C to almost any GPIO
  I2C_BUS_1.begin(I2C1_SDA, I2C1_SCL, 400000); // 400kHz Fast Mode
  I2C_BUS_2.begin(I2C2_SDA, I2C2_SCL, 400000); 

  // 3. Initialize Analog Pins
  analogReadResolution(12); // 12-bit resolution (0-4095)
  for (int i = 0; i < 10; i++) {
    pinMode(flexPins[i], INPUT);
  }

  // 4. Initialize IMUs
  Serial.println("Initializing 11 IMUs...");
  
  for (int i = 0; i < TOTAL_IMUS; i++) {
    // Select the correct bus and channel for this IMU
    tcaSelect(imuMapping[i].wireBus, imuMapping[i].muxChannel);
    
    // Attempt to initialize MPU6050
    // We pass the specific Wire bus to the .begin() function
    if (!mpus[i].begin(0x68, imuMapping[i].wireBus)) {
      Serial.print("Failed to find MPU on IMU index "); Serial.println(i);
    } else {
      // Configure settings for speed
      mpus[i].setAccelerometerRange(MPU6050_RANGE_8_G);
      mpus[i].setGyroRange(MPU6050_RANGE_500_DEG);
      mpus[i].setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
  }
  Serial.println("Setup Complete.");
}

void loop() {
  // Create a JSON document (Adjust size based on data complexity)
  // 11 IMUs * 6 floats + 10 Flex ints ~= significant data
  DynamicJsonDocument doc(4096);

  // --- 1. Read Flex Sensors ---
  JsonArray flexArray = doc.createNestedArray("flex");
  for (int i = 0; i < 10; i++) {
    flexArray.add(analogRead(flexPins[i]));
  }

  // --- 2. Read IMUs ---
  JsonArray imuArray = doc.createNestedArray("imu");
  
  for (int i = 0; i < TOTAL_IMUS; i++) {
    tcaSelect(imuMapping[i].wireBus, imuMapping[i].muxChannel);
    
    sensors_event_t a, g, temp;
    
    // We wrap this in a check because if an IMU failed setup, getEvent might hang
    // Note: In production, track which IMUs are valid boolean flags
    if (mpus[i].getEvent(&a, &g, &temp)) {
      JsonObject imuObj = imuArray.createNestedObject();
      // Round to 2 decimal places to save bandwidth
      imuObj["ax"] = (int)(a.acceleration.x * 100) / 100.0;
      imuObj["ay"] = (int)(a.acceleration.y * 100) / 100.0;
      imuObj["az"] = (int)(a.acceleration.z * 100) / 100.0;
      imuObj["gx"] = (int)(g.gyro.x * 100) / 100.0;
      imuObj["gy"] = (int)(g.gyro.y * 100) / 100.0;
      imuObj["gz"] = (int)(g.gyro.z * 100) / 100.0;
    } else {
      // Push null or error code if sensor read fails
      imuArray.add("ERR"); 
    }
  }

  // --- 3. Serialize and Send ---
  // If Bluetooth is connected, send data
  if (SerialBT.hasClient()) {
    serializeJson(doc, SerialBT);
    SerialBT.println(); // Newline delimiter
  } else {
    // Fallback to USB serial for debugging
    serializeJson(doc, Serial);
    Serial.println();
  }

  // Delay to manage sample rate (e.g., 50Hz = 20ms)
  delay(20);
}