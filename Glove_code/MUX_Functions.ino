void muxDisableAll() {
  // disable all channels on both muxes
  Wire.beginTransmission(TCA1_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(TCA2_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

void muxSelect(uint8_t muxIndex, uint8_t channel) {
  muxDisableAll();                 // ensure a single physical MPU is visible

  uint8_t addr = (muxIndex == 0) ? TCA1_ADDR : TCA2_ADDR;
  uint8_t mask = (1 << channel);   // channel 0..7 -> bitmask

  Wire.beginTransmission(addr);
  Wire.write(mask);
  Wire.endTransmission();
}

void selectMPU(uint8_t sensorID) {
  // simple example: first 8 sensors on mux A, next 8 on mux B
  uint8_t muxIndex = (sensorID < 8) ? 0 : 1;
  uint8_t channel  = sensorID % 8;

  muxSelect(muxIndex, channel);
}

void readDMPForSensor(uint8_t sensorID) {
  if (!dmpReady) return;

  selectMPU(sensorID);               // route bus to that MPU

  // NOTE: if you use FIFO + interrupts per sensor, you’d normally have
  // separate INT pins; in a muxed system you often poll FIFO instead.[web:72][web:80]

  fifoCount = IMU.getFIFOCount();    // bytes currently in FIFO[web:77]
  if (fifoCount < packetSize) return;

  // Handle potential overflow
  if (fifoCount >= 1024) {
    IMU.resetFIFO();
    return;
  }

  // Read one packet
  IMU.getFIFOBytes(fifoBuffer, packetSize);   // pop a DMP packet[web:77]

  // Extract quaternion, gravity vector, yaw/pitch/roll
  IMU.dmpGetQuaternion(&q, fifoBuffer);       // q = [w,x,y,z][web:74][web:77]
  IMU.dmpGetGravity(&gravity, &q);            // derived gravity[web:74][web:77]
  IMU.dmpGetYawPitchRoll(ypr, &q, &gravity);  // radians[web:74][web:77]

  Serial.print(sensorID);
  Serial.print(": ypr[");
  Serial.print(ypr[0] * 180.0 / M_PI); Serial.print(", ");
  Serial.print(ypr[1] * 180.0 / M_PI); Serial.print(", ");
  Serial.print(ypr[2] * 180.0 / M_PI); Serial.println("]");
}