#include <Arduino.h>
#include <sSense-HDC2010.h>


//I2C Addres van temp sesnor
#define ADDR 0x40

//Pinnen definen van hall sensor
#define hallDisablePin GPIO_NUM_4
#define hallOutPin GPIO_NUM_36

//Init temp sensor object en data var's
HDC2010 sensor(ADDR);
float temperature = 0, humidity = 0;

//RTC geheugen var om bij te houden als het systeem uit deepsleep kwam
RTC_DATA_ATTR int bootCount = 0;

//Testfunctie die alle registers van de temp sensor print
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


//setup indien systeem volledig herstart is
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


//Setup als systeem uit deepsleep komt
void wakeUpRoutine()
{
  temperature = sensor.readTemp();
  humidity = sensor.readHumidity();
  Serial.print("TEMP(C): ");
  Serial.println(temperature);
  Serial.print("%RH: ");
  Serial.println(humidity);
}

//Methode die de hallsensor uitleest
void readHallSensor(){
  uint16_t hallDrempelL = 1000;
  uint16_t hallDrempelH = 200;

  
  pinMode(hallDisablePin, OUTPUT);        //Pin als output zetten
  gpio_hold_dis(hallDisablePin);          // Disable hold functie -> sensor komt in active mode
  int hallData = analogRead(hallOutPin);  //Lees analoge waarde van sensor
  digitalWrite(hallDisablePin, HIGH);     //Zet de enable pin van de sensor hoog -> sensor komt in slaap mode
  gpio_deep_sleep_hold_en();              //Enable de hold functie tijdens deepsleep
  gpio_hold_en(hallDisablePin);           //Enable de hold functie voor bepaalde pin -> voorkomt dat pin laag komt als de esp slaapt!!

  //Threshold logica
  if (hallData <= hallDrempelL || hallData >=hallDrempelH)
    Serial.println("Deur open!!");
}


//Main setup
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  sensor.begin();

  //Logica die kijkt indien het systeem uit slaap komt of als het voor het eerst start
  if (bootCount == 0)
    firstBoot();
  else
    wakeUpRoutine();

  readHallSensor();

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1);   //Selecteer wake-up pin
  Serial.println("Going to sleep now");
  //Slaap
  esp_deep_sleep_start();
}

void loop()
{
  //Never in loop!!
}
