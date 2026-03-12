#include <Wire.h> // Include Wire library for I2C
#include <SparkFun_MMA8452Q.h> // Include SparkFun MMA8452Q library
// Define I2C addresses
#define MMA8452Q_ADDRESS_1 0x1C // Modified sensor (SA0 low)
#define MMA8452Q_ADDRESS_2 0x1D // Default sensor (SA0 high)
// Create two instances
MMA8452Q accel1(MMA8452Q_ADDRESS_1); // First sensor
MMA8452Q accel2(MMA8452Q_ADDRESS_2); // Second sensor

// Low-pass filter constant (alpha between 0 and 1; smaller alpha means more smoothing)
const float alpha = 0.1;

// Filtered values for sensor 1
float filtered_cx1 = 0.0;
float filtered_cy1 = 0.0;
float filtered_cz1 = 0.0;

// Filtered values for sensor 2
float filtered_cx2 = 0.0;
float filtered_cy2 = 0.0;
float filtered_cz2 = 0.0;

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor
  Serial.println("XC3732 Dual Sensor Test (MMA8452Q)");
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
void loop() {
  char acc1_msg[64]; // Buffer for sensor 1 string
  char acc2_msg[64]; // Buffer for sensor 2 string
 
  // Read from Sensor 1
  if (accel1.available()) {
    accel1.read();
    
    // Apply low-pass filter to accelerations
    filtered_cx1 = alpha * accel1.cx + (1 - alpha) * filtered_cx1;
    filtered_cy1 = alpha * accel1.cy + (1 - alpha) * filtered_cy1;
    filtered_cz1 = alpha * accel1.cz + (1 - alpha) * filtered_cz1;
    
    char x_buf[8], y_buf[8], z_buf[8], p_buf[8], r_buf[8];
    dtostrf(filtered_cx1, 6, 2, x_buf); // 6 chars, 2 decimal places
    dtostrf(filtered_cy1, 6, 2, y_buf);
    dtostrf(filtered_cz1, 6, 2, z_buf);
    float pitch = asin(filtered_cx1) * (180.0 / PI); // Pitch in degrees (x-axis tilt)
    float roll = atan2(filtered_cy1, filtered_cz1) * (180.0 / PI); // Roll in degrees (y/z tilt)
    dtostrf(pitch, 6, 2, p_buf);
    dtostrf(roll, 6, 2, r_buf);
   
    snprintf(acc1_msg, sizeof(acc1_msg), "S1_X %s,S1_Y %s,S1_Z %s,S1_p %s, S1_r %s",
             x_buf, y_buf, z_buf, p_buf, r_buf);
  }
  else {
    strcpy(acc1_msg, "S1_X nan,S1_Y nan,S1_Z nan,S1_p nan, S1_r nan");
  }
  // Read from Sensor 2
  if (accel2.available()) {
    accel2.read();
    
    // Apply low-pass filter to accelerations
    filtered_cx2 = alpha * accel2.cx + (1 - alpha) * filtered_cx2;
    filtered_cy2 = alpha * accel2.cy + (1 - alpha) * filtered_cy2;
    filtered_cz2 = alpha * accel2.cz + (1 - alpha) * filtered_cz2;
    
    char x_buf[8], y_buf[8], z_buf[8], p_buf[8], r_buf[8];
    dtostrf(filtered_cx2, 6, 2, x_buf); // 6 chars, 2 decimal places
    dtostrf(filtered_cy2, 6, 2, y_buf);
    dtostrf(filtered_cz2, 6, 2, z_buf);
    float pitch = asin(filtered_cx2) * (180.0 / PI); // Pitch in degrees (x-axis tilt)
    float roll = atan2(filtered_cy2, filtered_cz2) * (180.0 / PI); // Roll in degrees (y/z tilt)
    dtostrf(pitch, 6, 2, p_buf);
    dtostrf(roll, 6, 2, r_buf);
   
    snprintf(acc2_msg, sizeof(acc2_msg), "S2_X %s,S2_Y %s,S2_Z %s,S2_p %s, S2_r %s",
             x_buf, y_buf, z_buf, p_buf, r_buf);
  }
  else {
    strcpy(acc2_msg, "S2_X nan,S2_Y nan,S2_Z nan,S2_p nan, S2_r nan");
  }
  // Combine messages
  char message[128];
  snprintf(message, sizeof(message), "%s,%s", acc1_msg, acc2_msg);
  Serial.println(message);
  delay(500); // Update every 500ms
}
// Get orientation as String
String getOrientation(MMA8452Q &accel) {
  byte pl = accel.readPL();
  switch (pl) {
    case PORTRAIT_U: return "1"; //Portrait Up
    case PORTRAIT_D: return "2"; //Portrait Down
    case LANDSCAPE_R: return "3"; //Landscape Right
    case LANDSCAPE_L: return "4"; //Landscape Left
    case LOCKOUT: return "Flat";
    default: return "Unknown";
  }
}
