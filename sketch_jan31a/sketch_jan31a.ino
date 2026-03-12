#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ADC pins dedicated to flex sensors
#DEFINE LpinkMid 13
#DEFINE LpinkProx 12

#DEFINE LringMid 11
#DEFINE LringProx 10

#DEFINE LmiddMid 9
#DEFINE LmiddProx 8

#DEFINE LindeMid 7
#DEFINE LindeProx 16

#DEFINE LthumMid 15
#DEFINE LthumProx 14

// Address select pins for the the multiplexors
#DEFINE Mux1_AddrPin1 45
#DEFINE Mux1_AddrPin2 42
#DEFINE Mux1_AddrPin3 41

#DEFINE Mux2_AddrPin1 40
#DEFINE Mux2_AddrPin2 39
#DEFINE Mux2_AddrPin3 38


// Device Addresses
#define TCA_ADDR 0x70  // Default address for TCA9548A [6]
#define MPU_ADDR 0x68  // Default address for MPU-6050 [5]


void setup() {
  // put your setup code here, to run once:
  pinMode(LpinkMid, INPUT);
  pinMode(LringMid, INPUT);
  pinMode(LmiddMid, INPUT);
  pinMode(LindeMid, INPUT);
  pinMode(LthumMid, INPUT);

  pinMode(LpinkProx, INPUT);
  pinMode(LringProx, INPUT);
  pinMode(LmiddProx, INPUT);
  pinMode(LindeProx, INPUT);
  pinMode(LthumProx, INPUT);

  pinMode(Mux1_AddrPin1, OUTPUT);
  pinMode(Mux1_AddrPin2, OUTPUT);
  pinMode(Mux1_AddrPin3, OUTPUT);

  pinMode(Mux2_AddrPin1, OUTPUT);
  pinMode(Mux2_AddrPin2, OUTPUT);
  pinMode(Mux2_AddrPin3, OUTPUT);

}


void loop() {
  // put your main code here, to run repeatedly:

  for (int i=0; i<8; i++){

    digitalWrite(Mux1_AddrPin1, (i[0] == 1) ? true : false);
    digitalWrite(Mux1_AddrPin2, (i[1] == 1) ? true : false);
    digitalWrite(Mux1_AddrPin3, (i[2] == 1) ? true : false);

    digitalWrite(Mux2_AddrPin1, (i[0] == 1) ? true : false);
    digitalWrite(Mux2_AddrPin2, (i[1] == 1) ? true : false);
    digitalWrite(Mux2_AddrPin3, (i[2] == 1) ? true : false);
  }


}
