#include <Wire.h>

// I2C Addresses
const uint8_t TCA_ADDR = 0x70;  // Default address for TCA9548A [1]
const uint8_t MPU_ADDR = 0x68;  // Default address for MPU-6050 [2]

const int TCA_ADDR_PIN_0 = 7;
const int TCA_ADDR_PIN_1 = 9;
const int TCA_ADDR_PIN_2 = 13;

void setup() {
  Serial.begin(115200);
  pinMode(TCA_ADDR_PIN_0, OUTPUT);
  pinMode(TCA_ADDR_PIN_1, OUTPUT);
  pinMode(TCA_ADDR_PIN_2, OUTPUT);
  
  digitalWrite(TCA_ADDR_PIN_0, LOW);
  digitalWrite(TCA_ADDR_PIN_1, LOW);
  digitalWrite(TCA_ADDR_PIN_2, LOW);
  
  // Initialize I2C on the ESP32-C6 [3, 4]
  // Default pins for ESP32-C6 are often SDA=6, SCL=7 based on common DevKit layouts
  Wire.begin(); 

  Serial.println("Initializing MPU-6050s on all channels...");

  // Iteratively initialize each MPU-6050 connected to the 8 channels [5]
  for (uint8_t i = 0; i < 8; i++) {
    tca_select(i);
    
    // Wake up the MPU-6050 [6]
    // PWR_MGMT_1 register is 0x6B. Writing 0 wakes it up.
    // (Note: Register 0x6B info is standard MPU-6050 operation not explicitly in sources)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); 
    Wire.write(0);    
    if (Wire.endTransmission() == 0) {
      Serial.print("MPU-6050 found and initialized on Channel ");
      Serial.println(i);
    } else {
      Serial.print("No MPU-6050 found on Channel ");
      Serial.println(i);
    }
  }
}

void loop() {
  // Iteratively communicate with each channel [5]
  for (uint8_t i = 0; i < 8; i++) {
    tca_select(i);
    read_mpu_data(i);
    delay(100); // Small delay between channel reads
  }
  delay(1000); // Pause before next full iteration
}

/**
 * Function to select the active channel on the TCA9548A [7]
 * Writing a byte where bit 'n' is high enables channel 'n' [8]
 */
void tca_select(uint8_t channel) {
  if (channel > 7) return;
  
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel); // Set bit for the desired channel [8, 9]
  Wire.endTransmission();
}

/**
 * Function to read accelerometer data from the MPU-6050 [10]
 */
void read_mpu_data(uint8_t channel) {
  // Starting register for Accelerometer data is 0x3B (standard for MPU-6050)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  if (Wire.endTransmission(false) != 0) return;

  // Request 6 bytes (X, Y, Z for Accelerometer) [10, 11]
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  
  if (Wire.available() == 6) {
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();

    Serial.print("CH");
    Serial.print(channel);
    Serial.print(" - Accel X: "); Serial.print(ax);
    Serial.print(" Y: "); Serial.print(ay);
    Serial.print(" Z: "); Serial.println(az);
  }
}