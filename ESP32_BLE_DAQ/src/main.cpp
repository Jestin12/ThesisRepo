#include <Arduino.h>

// On many ESP32-S3 DevKitM-1 boards the built-in LED is on GPIO 48.
// If your board uses another pin, change this accordingly.
#ifndef LED_BUILTIN
#define LED_BUILTIN 48
#endif

void setup() {
  // Small delay so ROM boot messages clear
  delay(500);

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("===== Minimal sanity check sketch started =====");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.print("LED pin is ");
  Serial.println(LED_BUILTIN);
}

void loop() {
  Serial.println("Toggling LED...");
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(500);
}
