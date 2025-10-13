#include <Wire.h>
#include <math.h>

// MMA8452Q I2C addresses
#define MMA8452Q1_ADDRESS 0x1C // Sensor 1 (SA0 low)
#define MMA8452Q2_ADDRESS 0x1D // Sensor 2 (SA0 high)

// Sensor data structure
struct SensorData {
  int16_t x, y, z;      // Raw acceleration values
  float angle;          // Angle in degrees
  float velocity;       // Angular velocity in deg/s
  float acceleration;   // Magnitude of acceleration in g
  float lastAngle;      // Previous angle for velocity calculation
  unsigned long lastTime; // Last measurement time
};

// Global sensor data variables
SensorData sensor1 = {0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0};
SensorData sensor2 = {0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0};

// Initialize MMA8452Q sensor
bool setupMMA8452Q(uint8_t address) {
  // Verify sensor presence
  Wire.beginTransmission(address);
  Wire.write(0x0D); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.available() != 1 || Wire.read() != 0x2A) { // Expected WHO_AM_I value for MMA8452Q
    return false;
  }

  // Configure sensor: 2g range, 800 Hz data rate, active mode
  Wire.beginTransmission(address);
  Wire.write(0x0E); // XYZ_DATA_CFG
  Wire.write(0x00); // 2g range
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(0x2A); // CTRL_REG1
  Wire.write(0x05); // 800 Hz, active mode
  Wire.endTransmission();

  return true;
}

// Read data from MMA8452Q sensor
bool readMMA8452Q(uint8_t address, SensorData &data) {
  Wire.beginTransmission(address);
  Wire.write(0x01); // OUT_X_MSB
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)6);
  if (Wire.available() != 6) {
    return false;
  }

  // Read 12-bit data (right-justified)
  data.x = (Wire.read() << 8) | Wire.read();
  data.y = (Wire.read() << 8) | Wire.read();
  data.z = (Wire.read() << 8) | Wire.read();
  data.x >>= 4; // Convert to 12-bit
  data.y >>= 4;
  data.z >>= 4;

  // Scale to g (2g range, 12-bit: 4096 counts/g)
  const float scale = 1.0 / 4096.0;
  data.acceleration = sqrt(data.x * data.x + data.y * data.y + data.z * data.z) * scale;

  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize sensors
//  if (!setupMMA8452Q(MMA8452Q1_ADDRESS)) {
//    Serial.println("Error: Failed to initialize Sensor 1");
//    while (1); // Halt on error
//  }
  while (!setupMMA8452Q(MMA8452Q1_ADDRESS)){
    Serial.println("Error: Failed to initialize Sensor 1");
  }
  
//  if (!setupMMA8452Q(MMA8452Q2_ADDRESS)) {
//    Serial.println("Error: Failed to initialize Sensor 2");
//    while (1); // Halt on error
//  }
  while (!setupMMA8452Q(MMA8452Q2_ADDRESS)){
    Serial.println("Error: Failed to initialize Sensor 2");
  }

  // Initialize timing
  sensor1.lastTime = millis();
  sensor2.lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt1 = (now - sensor1.lastTime) / 1000.0;
  float dt2 = (now - sensor2.lastTime) / 1000.0;

  // Read and process Sensor 1
  if (readMMA8452Q(MMA8452Q1_ADDRESS, sensor1)) {
    sensor1.angle = atan2(sensor1.y, sensor1.x) * 180.0 / M_PI;
    sensor1.velocity = (dt1 > 0) ? (sensor1.angle - sensor1.lastAngle) / dt1 : 0.0;
    sensor1.lastAngle = sensor1.angle;
    sensor1.lastTime = now;
  } else {
    Serial.println("Error: Failed to read Sensor 1");
  }

  // Read and process Sensor 2
  if (readMMA8452Q(MMA8452Q2_ADDRESS, sensor2)) {
    sensor2.angle = atan2(sensor2.y, sensor2.x) * 180.0 / M_PI;
    sensor2.velocity = (dt2 > 0) ? (sensor2.angle - sensor2.lastAngle) / dt2 : 0.0;
    sensor2.lastAngle = sensor2.angle;
    sensor2.lastTime = now;
  } else {
    Serial.println("Error: Failed to read Sensor 2");
  }

  // Output results
  Serial.print("Sensor1 - Angle: "); Serial.print(sensor1.angle, 2);
  Serial.print(" deg, Velocity: "); Serial.print(sensor1.velocity, 2);
  Serial.print(" deg/s, Accel: "); Serial.print(sensor1.acceleration, 2);
  Serial.print(" g | Sensor2 - Angle: "); Serial.print(sensor2.angle, 2);
  Serial.print(" deg, Velocity: "); Serial.print(sensor2.velocity, 2);
  Serial.print(" deg/s, Accel: "); Serial.print(sensor2.acceleration, 2);
  Serial.println(" g");

  delay(100); // 100ms delay for ~10 Hz update rate
}
