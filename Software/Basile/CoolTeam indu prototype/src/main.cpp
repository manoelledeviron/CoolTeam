#include <Arduino.h>
#include <sSense-HDC2010.h>
#include <Adafruit_GPS.h>



//I2C Addres van temp sensor en GPS
#define ADDR 0x40
#define GPSADDR 0x10

//Pinnen definen van hall sensor
#define hallDisablePin GPIO_NUM_4
#define hallOutPin GPIO_NUM_36

//Pinnen definieren van GPS
#define GpsEnable GPIO_NUM_32

//Init temp sensor object en data var's
HDC2010 sensor(ADDR);
float temperature = 0, humidity = 0;

//Init GPS data variabelen
bool doorOpen = false;
bool isInitiatedGPS = false;
Adafruit_GPS GPS(&Wire);
uint32_t timer = millis();
bool gotLocation = false;
#define GPSECHO false //voor debugging en printen van raw GPS data

nmea_float_t gpsLat;
nmea_float_t gpsLatNS;
nmea_float_t gpsLon;
nmea_float_t gpsLonEW;
nmea_float_t gpsAlt;

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

  //Serial.println("First boot!");
}


//Setup als systeem uit deepsleep komt
void wakeUpRoutine()
{
  temperature = sensor.readTemp();
  humidity = sensor.readHumidity();
  //Serial.print("TEMP(C): ");
  //Serial.println(temperature);
  //Serial.print("%RH: ");
  //Serial.println(humidity);
}

void initGPS() 
{
  GPS.begin(GPSADDR);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //GPS mode: RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);

  // Ask for firmware version
  GPS.println(PMTK_Q_RELEASE);
  isInitiatedGPS = true;
}

void readGPS()
{
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    if (GPS.fix) {
      gpsLat = GPS.latitude; gpsLatNS = GPS.lat;
      gpsLon = GPS.longitude; gpsLonEW = GPS.lon;
      gpsAlt = GPS.altitude;
      gotLocation = true;

      /*
      // print location
      Serial.print("Location: ");
      Serial.print(gpsLat, 4); Serial.print(gpsLatNS);
      Serial.print(", ");
      Serial.print(gpsLon, 4); Serial.println(gpsLonEW);
      Serial.print("Altitude: "); Serial.println(gpsAlt);
      */
    }
  }
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
    if (!isInitiatedGPS){
      digitalWrite(GpsEnable,HIGH);
      initGPS();
    }
    while (!gotLocation){
      readGPS();
    }
    //stuur gpsLat, gpsLatNS, gpsLon, gpsLonEW, gpsAlt door via LoRa
}

//Main setup
void setup()
{
  //Serial.begin(115200);
  //while (!Serial)
  //  ;
  sensor.begin();

  //Logica die kijkt indien het systeem uit slaap komt of als het voor het eerst start
  if (bootCount == 0)
    firstBoot();
  else
    wakeUpRoutine();

  //readHallSensor();

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1);   //Selecteer wake-up pin
  //Serial.println("Going to sleep now");
  //Slaap
  esp_deep_sleep_start();
}

void loop()
{
  //Never in loop!!
}
