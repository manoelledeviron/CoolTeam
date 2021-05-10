#include <Arduino.h>
const uint8_t io1 = 14;
const uint8_t io2 = 15;
const uint8_t io3 = 2;
void setup() {
  // put your setup code here, to run once:
  pinMode(io1, OUTPUT);
  pinMode(io2, OUTPUT);
  pinMode(io3, OUTPUT);
  Serial.begin(9600);
  Serial.println("START");
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(io2,HIGH);
  digitalWrite(io1,HIGH);
  Serial.println("AAN");
  delay(1000);
  digitalWrite(io1,LOW);
  Serial.println("UIT");
  delay(1000);
}