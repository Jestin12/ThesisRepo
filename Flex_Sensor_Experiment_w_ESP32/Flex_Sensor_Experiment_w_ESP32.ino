const int adcPin = 2;  // change to your ADC pin

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
}

void loop() {
  // Read ADC (millivolts)
  int analogVolts = analogReadMilliVolts(adcPin);

  // Print just the ADC value
  Serial.println(analogVolts);

  // Adjust sampling rate as needed
  delay(1);  // 1 ms between samples
}
