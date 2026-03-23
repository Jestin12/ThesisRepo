/**
 * MPU6050 IMU Interface — ESP32-S3
 *
 * Reads raw accelerometer and gyroscope data from a single MPU6050
 * and outputs JSON over USB Serial.
 *
 * Wiring:
 *   MPU6050 SDA  ->  ESP32-S3 GPIO 11
 *   MPU6050 SCL  ->  ESP32-S3 GPIO 10
 *   MPU6050 VCC  ->  3.3V
 *   MPU6050 GND  ->  GND
 *   MPU6050 AD0  ->  GND  (I2C address = 0x68)
 *
 * Library: Adafruit MPU6050  (install via Library Manager)
 * Depends: Adafruit Unified Sensor, Adafruit BusIO
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ── Configuration ─────────────────────────────────────────────────────────────

#define I2C_SDA       9
#define I2C_SCL       8
#define I2C_FREQ      400000UL   // 400 kHz fast mode

#define MPU_ADDR      0x68       // AD0 low = 0x68 | AD0 high = 0x69
#define SAMPLE_RATE_HZ 100       // Output rate in Hz
#define SAMPLE_INTERVAL_MS (1000 / SAMPLE_RATE_HZ)

// Accelerometer full-scale range: 2G / 4G / 8G / 16G
#define ACCEL_RANGE   MPU6050_RANGE_16_G

// Gyroscope full-scale range: 250 / 500 / 1000 / 2000 deg/s
#define GYRO_RANGE    MPU6050_RANGE_2000_DEG

// Digital low-pass filter bandwidth
#define DLPF_BW       MPU6050_BAND_10_HZ

// ── Globals ───────────────────────────────────────────────────────────────────

Adafruit_MPU6050 mpu;

unsigned long lastSampleTime = 0;
uint32_t sampleIndex = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);   // Wait for USB CDC to enumerate

  // Initialise I2C on custom pins
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

  Serial.println("[INFO] Initialising MPU6050...");

  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("[ERROR] MPU6050 not found. Check wiring and I2C address.");
    while (true) delay(100);   // Halt
  }

  // Configure sensor ranges and filter
  mpu.setAccelerometerRange(ACCEL_RANGE);
  mpu.setGyroRange(GYRO_RANGE);
  mpu.setFilterBandwidth(DLPF_BW);

  // Print confirmed configuration
  Serial.print("[INFO] Accel range : ±");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:  Serial.println("2G");   break;
    case MPU6050_RANGE_4_G:  Serial.println("4G");   break;
    case MPU6050_RANGE_8_G:  Serial.println("8G");   break;
    case MPU6050_RANGE_16_G: Serial.println("16G");  break;
  }
  Serial.print("[INFO] Gyro range  : ±");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:  Serial.println("250 deg/s");  break;
    case MPU6050_RANGE_500_DEG:  Serial.println("500 deg/s");  break;
    case MPU6050_RANGE_1000_DEG: Serial.println("1000 deg/s"); break;
    case MPU6050_RANGE_2000_DEG: Serial.println("2000 deg/s"); break;
  }
  Serial.print("[INFO] DLPF        : ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ: Serial.println("260 Hz"); break;
    case MPU6050_BAND_184_HZ: Serial.println("184 Hz"); break;
    case MPU6050_BAND_94_HZ:  Serial.println("94 Hz");  break;
    case MPU6050_BAND_44_HZ:  Serial.println("44 Hz");  break;
    case MPU6050_BAND_21_HZ:  Serial.println("21 Hz");  break;
    case MPU6050_BAND_10_HZ:  Serial.println("10 Hz");  break;
    case MPU6050_BAND_5_HZ:   Serial.println("5 Hz");   break;
  }

  Serial.println("[INFO] Ready. Streaming JSON...");
}

// ── Loop ──────────────────────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();

  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = now;

  // Fetch sensor events
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Build compact JSON — fields in m/s² and rad/s (SI units from Adafruit driver)
  // Use Serial.print chaining to avoid sprintf buffer overhead
  Serial.print("{\"t\":");
  Serial.print(now);
  Serial.print(",\"i\":");
  Serial.print(sampleIndex++);
  Serial.print(",\"ax\":");
  Serial.print(accel.acceleration.x, 4);
  Serial.print(",\"ay\":");
  Serial.print(accel.acceleration.y, 4);
  Serial.print(",\"az\":");
  Serial.print(accel.acceleration.z, 4);
  Serial.print(",\"gx\":");
  Serial.print(gyro.gyro.x, 4);
  Serial.print(",\"gy\":");
  Serial.print(gyro.gyro.y, 4);
  Serial.print(",\"gz\":");
  Serial.print(gyro.gyro.z, 4);
  Serial.println("}");
}
