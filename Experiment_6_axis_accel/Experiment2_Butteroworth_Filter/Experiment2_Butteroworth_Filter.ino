#include <Wire.h> // Include Wire library for I2C
#include <SparkFun_MMA8452Q.h> // Include SparkFun MMA8452Q library
// Define I2C addresses
#define MMA8452Q_ADDRESS_1 0x1C // Modified sensor (SA0 low)
#define MMA8452Q_ADDRESS_2 0x1D // Default sensor (SA0 high)
// Create two instances
MMA8452Q accel1(MMA8452Q_ADDRESS_1); // First sensor
MMA8452Q accel2(MMA8452Q_ADDRESS_2); // Second sensor

// Butterworth low-pass filter parameters
// Second-order Butterworth low-pass filter with cutoff frequency fc = 5 Hz
// Sampling frequency fs = 2 Hz (based on 500 ms delay, effective rate after available check ≈800 Hz but smoothed output at 2 Hz)
// For stability and noise reduction in gesture recognition, fc = 5 Hz is suitable for smoothing hand movements while preserving dynamics
const float fs = 2.0;      // Effective output sampling frequency (Hz)
const float fc = 5.0;      // Cutoff frequency (Hz)
const float omega = tan(PI * fc / fs);
const float omega_sq = omega * omega;
const float denom = omega_sq + sqrt(2) * omega + 1;
const float b0 = omega_sq / denom;
const float b1 = 2 * b0;
const float b2 = b0;
const float a1 = 2 * (omega_sq - 1) / denom;
const float a2 = (1 - sqrt(2) * omega + omega_sq) / denom;

// Filter history for sensor 1 (x, y, z)
float x1_1 = 0.0, x1_2 = 0.0;  // Input history
float y1_1_x = 0.0, y1_2_x = 0.0;  // Output history for x
float y1_1_y = 0.0, y1_2_y = 0.0;  // Output history for y
float y1_1_z = 0.0, y1_2_z = 0.0;  // Output history for z

// Filter history for sensor 2 (x, y, z)
float x2_1 = 0.0, x2_2 = 0.0;
float y2_1_x = 0.0, y2_2_x = 0.0;
float y2_1_y = 0.0, y2_2_y = 0.0;
float y2_1_z = 0.0, y2_2_z = 0.0;

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor
  Serial.println("XC3732 Dual Sensor Test (MMA8452Q) with Butterworth Filter");
  Wire.begin(); // Initialize I2C
  // Initialize first sensor
  if (accel1.init(SCALE_2G, ODR_800)) {
    Serial.println("XC3732 Sensor 1 (0x1C) initialized");
  } else {
    Serial.println("Failed to initialize XC3732 Sensor 1");
    while (1);
  }
  // Initialize second sensor
  if (accel2.init(SCALE_2G, ODR_800)) {
    Serial.println("XC3732 Sensor 2 (0x1D) initialized");
  } else {
    Serial.println("Failed to initialize XC3732 Sensor 2");
    while (1);
  }
}

float apply_butterworth(float input, float& x1, float& x2, float& y1, float& y2) {
  // Shift input history
  x2 = x1;
  x1 = input;
  
  // Compute output
  float output = b0 * x1 + b1 * x2 + b2 * x2 - a1 * y1 - a2 * y2;  // Note: x2 used twice for b1 and b2 symmetry
  
  // Shift output history
  y2 = y1;
  y1 = output;
  
  return output;
}

void loop() {
  char acc1_msg[64]; // Buffer for sensor 1 string
  char acc2_msg[64]; // Buffer for sensor 2 string
 
  // Read from Sensor 1
  if (accel1.available()) {
    accel1.read();
    
    // Apply Butterworth filter to each axis
    float filtered_cx = apply_butterworth(accel1.cx, x1_1, x1_2, y1_1_x, y1_2_x);
    float filtered_cy = apply_butterworth(accel1.cy, x1_1, x1_2, y1_1_y, y1_2_y);
    float filtered_cz = apply_butterworth(accel1.cz, x1_1, x1_2, y1_1_z, y1_2_z);
    
    char x_buf[8], y_buf[8], z_buf[8], p_buf[8], r_buf[8];
    dtostrf(filtered_cx, 6, 2, x_buf);
    dtostrf(filtered_cy, 6, 2, y_buf);
    dtostrf(filtered_cz, 6, 2, z_buf);
    float pitch = asin(filtered_cx) * (180.0 / PI); // Pitch in degrees
    float roll = atan2(filtered_cy, filtered_cz) * (180.0 / PI); // Roll in degrees
    dtostrf(pitch, 6, 2, p_buf);
    dtostrf(roll, 6, 2, r_buf);
   
    snprintf(acc1_msg, sizeof(acc1_msg), "S1_X %s,S1_Y %s,S1_Z %s,S1_p %s,S1_r %s",
             x_buf, y_buf, z_buf, p_buf, r_buf);
  }
  else {
    strcpy(acc1_msg, "S1_X nan,S1_Y nan,S1_Z nan,S1_p nan,S1_r nan");
  }
  
  // Read from Sensor 2
  if (accel2.available()) {
    accel2.read();
    
    // Apply Butterworth filter to each axis
    float filtered_cx = apply_butterworth(accel2.cx, x2_1, x2_2, y2_1_x, y2_2_x);
    float filtered_cy = apply_butterworth(accel2.cy, x2_1, x2_2, y2_1_y, y2_2_y);
    float filtered_cz = apply_butterworth(accel2.cz, x2_1, x2_2, y2_1_z, y2_2_z);
    
    char x_buf[8], y_buf[8], z_buf[8], p_buf[8], r_buf[8];
    dtostrf(filtered_cx, 6, 2, x_buf);
    dtostrf(filtered_cy, 6, 2, y_buf);
    dtostrf(filtered_cz, 6, 2, z_buf);
    float pitch = asin(filtered_cx) * (180.0 / PI); // Pitch in degrees
    float roll = atan2(filtered_cy, filtered_cz) * (180.0 / PI); // Roll in degrees
    dtostrf(pitch, 6, 2, p_buf);
    dtostrf(roll, 6, 2, r_buf);
   
    snprintf(acc2_msg, sizeof(acc2_msg), "S2_X %s,S2_Y %s,S2_Z %s,S2_p %s,S2_r %s",
             x_buf, y_buf, z_buf, p_buf, r_buf);
  }
  else {
    strcpy(acc2_msg, "S2_X nan,S2_Y nan,S2_Z nan,S2_p nan,S2_r nan");
  }
  
  // Combine messages
  char message[128];
  snprintf(message, sizeof(message), "%s,%s", acc1_msg, acc2_msg);
  Serial.println(message);
  delay(500); // Update every 500 ms
}

// Get orientation as String (unchanged)
String getOrientation(MMA8452Q &accel) {
  byte pl = accel.readPL();
  switch (pl) {
    case PORTRAIT_U: return "1"; // Portrait Up
    case PORTRAIT_D: return "2"; // Portrait Down
    case LANDSCAPE_R: return "3"; // Landscape Right
    case LANDSCAPE_L: return "4"; // Landscape Left
    case LOCKOUT: return "Flat";
    default: return "Unknown";
  }
}
