#include <Arduino.h>
#include <rn2xx3.h>

rn2xx3 myLora(Serial2);
const uint8_t LoRaReset = 18;
const uint8_t LoRaEnable = 19;

void setup() {
  // put your setup code here, to run once:
  pinMode(LoRaEnable, OUTPUT);
  digitalWrite(LoRaEnable, HIGH);
  Serial.begin(9600); //Serial computer
  Serial.println("START");
  delay(1000);
  Serial2.begin(57600); //serial lora
  delay(1000);
  Serial.println("INITIALIZE");
  initialize_rn2483();
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void initialize_rn2483()
{
  //reset rn2483
  pinMode(LoRaReset, OUTPUT);
  digitalWrite(LoRaReset, LOW);
  delay(500);
  digitalWrite(LoRaReset, HIGH);
  delay(100); //wait for the RN2xx3's startup message
  Serial.println("REBOOTED");
  Serial2.flush();
  myLora.autobaud();

  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    delay(10000);
    hweui = myLora.hweui();
  }
  Serial.println(hweui);
  bool join_result = false;
  const char *appEui = "0000000000000000";
  const char *appKey = "153F68488BE1FE937F4ECA5E330FC4D6";
  const char *devEui = "F801000189040000";
  join_result = myLora.initOTAA(appEui, appKey,devEui);
  Serial.println("JOIN OTAA");
  while(!join_result)
  {
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  Serial.println("JOINED");
}