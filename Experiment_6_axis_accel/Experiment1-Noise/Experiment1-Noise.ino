#include <Wire.h>

#define MMA8452Q_ADDRESS 0x1C // Default I2C address

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Put MMA8452Q into active mode
  Wire.beginTransmission(MMA8452Q_ADDRESS);
  Wire.write(0x2A); // CTRL_REG1
  Wire.write(0x01); // Set ACTIVE bit
  Wire.endTransmission();
}

void loop() {
  int16_t x, y, z;

  Wire.beginTransmission(MMA8452Q_ADDRESS);
  Wire.write(0x01); // Start at OUT_X_MSB
  Wire.endTransmission(false);
  Wire.requestFrom(MMA8452Q_ADDRESS, 6);

  if (Wire.available() == 6) {
    x = (Wire.read() << 8) | Wire.read();
    y = (Wire.read() << 8) | Wire.read();
    z = (Wire.read() << 8) | Wire.read();

    // 12-bit values, so shift right by 4
    x >>= 4;
    y >>= 4;
    z >>= 4;

    Serial.print("X: "); Serial.print(x);
    Serial.print(" Y: "); Serial.print(y);
    Serial.print(" Z: "); Serial.println(z);
  } else {
    Serial.println("Error reading MMA8452Q");
  }

  delay(100);
}