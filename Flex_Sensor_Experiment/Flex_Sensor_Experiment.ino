// Read A0 and send "MM:SS.mmm,ADC" over UART every 10 ms

unsigned long startTime;

void setup() {
  pinMode(A0, INPUT);
  Serial.begin(115200);
  startTime = millis();
}

void loop() {
  unsigned long now  = millis() - startTime;     // elapsed ms
  unsigned long ms   = now % 1000;               // 0–999
  unsigned long sec  = now / 1000;
  unsigned long min  = sec / 60;

  sec %= 60;
  min %= 60;

  // "MM:SS.mmm"
  char timeStamp[13];  // "MM:SS.mmm" + '\0' => 11, give a bit extra
  snprintf(timeStamp, sizeof(timeStamp),
           "%02lu:%02lu.%03lu", min, sec, ms);   // zero‑padded mm:ss.mmm[web:46][web:83]

  int adc = analogRead(A0);

  Serial.print(timeStamp);
  Serial.print(",");
  Serial.println(adc);

  delay(10);
}
