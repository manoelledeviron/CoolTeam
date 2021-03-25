#include <Arduino.h>
#include <sSense-HDC2010.h>

#define ADDR 0x40

#define hallDisablePin GPIO_NUM_4
#define hallOutPin GPIO_NUM_36

HDC2010 sensor(ADDR);
float temperature = 0, humidity = 0;

RTC_DATA_ATTR int bootCount = 0;

void printRegisters()
{

  for (size_t i = 0; i < 16; i++)
  {
    Serial.print("Register ");
    Serial.print(i, HEX);
    Serial.print(": ");
    Serial.println(sensor.readReg(i), BIN);
  }
}

void firstBoot()
{
  sensor.reset();
  delay(3);
  sensor.setMeasurementMode(TEMP_AND_HUMID);
  sensor.setRate(FIVE_SECONDS);
  sensor.setTempRes(FOURTEEN_BIT);
  sensor.setHumidRes(FOURTEEN_BIT);
  sensor.setInterruptPolarity(ACTIVE_HIGH);
  sensor.enableInterrupt();
  sensor.enableDRDYInterrupt();
  sensor.triggerMeasurement();

  bootCount++;

  Serial.println("First boot!");
}

void wakeUpRoutine()
{
  temperature = sensor.readTemp();
  humidity = sensor.readHumidity();
  Serial.print("TEMP(C): ");
  Serial.println(temperature);
  Serial.print("%RH: ");
  Serial.println(humidity);
}

void readHallSensor(){
  uint16_t hallDrempelL = 1000;
  uint16_t hallDrempelH = 200;
  pinMode(hallDisablePin, OUTPUT);
  gpio_hold_dis(hallDisablePin);
  int hallData = analogRead(hallOutPin);
  digitalWrite(hallDisablePin, HIGH);
  gpio_deep_sleep_hold_en();
  gpio_hold_en(hallDisablePin);
  if (hallData <= hallDrempelL || hallData >=hallDrempelH)
    Serial.println("Deur open!!");
}

void setup()
{

  Serial.begin(115200);
  while (!Serial)
    ;
  sensor.begin();

  if (bootCount == 0)
    firstBoot();
  else
    wakeUpRoutine();


  readHallSensor();

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1);
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();

  //printRegisters();
}

void loop()
{
  //Never in loop!!
}
