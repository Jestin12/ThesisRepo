#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial to be ready

  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055! Check wiring or I2C address.");
    while (1);
  }

  bno.setExtCrystalUse(true);
}

void loop() {
  // Accelerometer
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // Gyroscope
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // Magnetometer
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  // Euler angles (heading, roll, pitch)
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("Accel (m/s^2): ");
  Serial.print(accel.x()); Serial.print(", ");
  Serial.print(accel.y()); Serial.print(", ");
  Serial.print(accel.z()); Serial.print(" | ");

  Serial.print("Gyro (deg/s): ");
  Serial.print(gyro.x()); Serial.print(", ");
  Serial.print(gyro.y()); Serial.print(", ");
  Serial.print(gyro.z()); Serial.print(" | ");

  Serial.print("Mag (uT): ");
  Serial.print(mag.x()); Serial.print(", ");
  Serial.print(mag.y()); Serial.print(", ");
  Serial.print(mag.z()); Serial.print(" | ");

  Serial.print("Euler (deg): ");
  Serial.print(euler.x()); Serial.print(", ");
  Serial.print(euler.y()); Serial.print(", ");
  Serial.print(euler.z());

  Serial.println();

  delay(100);
}
