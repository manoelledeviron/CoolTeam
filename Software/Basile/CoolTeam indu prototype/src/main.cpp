#include <Arduino.h>
#include <sSense-HDC2010.h>
#include <Adafruit_GPS.h>
#include <rn2xx3.h>
#include <esp_sleep.h>
#include <driver/uart.h>


//I2C Addres van temp sensor
#define ADDR 0x40

//Pinnen definen van hall sensor
#define hallDisablePin GPIO_NUM_4
#define hallOutPin GPIO_NUM_36

//Pinnen definieren van GPS
#define GpsEnable GPIO_NUM_5

//Init temp sensor object en data var's
HDC2010 sensor(ADDR);
float temperature = 0, humidity = 0;

//Init GPS data variabelen
bool isInitiatedGPS = false;
Adafruit_GPS GPS(&Wire);
uint32_t timer = millis();
bool gotLocation = false;
bool GPSECHO =true; //voor debugging en printen van raw GPS data

nmea_float_t gpsLat;
nmea_float_t gpsLatNS;
nmea_float_t gpsLon;
nmea_float_t gpsLonEW;
nmea_float_t gpsAlt;

//RTC geheugen var om bij te houden als het systeem uit deepsleep kwam
RTC_DATA_ATTR int bootCount = 0;

//LoRa variabelen
rn2xx3 myLora(Serial2);
#define LoRaEnable GPIO_NUM_19
#define LoRaReset GPIO_NUM_18
#define MINUTES_SEND 1
RTC_DATA_ATTR int temptimer = 0;



//init van de LoRa. NA IEDERE POWER UP NODIG.
void init_rn2483()
{
  //spanning voorzien van de Lora
  pinMode(LoRaEnable, OUTPUT);
  digitalWrite(LoRaEnable, HIGH);
  //reset rn2483
  pinMode(LoRaReset, OUTPUT);
  digitalWrite(LoRaReset, LOW);
  sleep_time(500);
  digitalWrite(LoRaReset, HIGH);
  sleep_time(100); //wait for the RN2xx3's startup message
  Serial2.flush();
  myLora.autobaud();
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    sleep_time(1000);
    hweui = myLora.hweui();
  }
  bool join_result = false;
  const char *appEui = "0000000000000000";
  const char *appKey = "153F68488BE1FE937F4ECA5E330FC4D6";
  const char *devEui = "F801000189040000";
  join_result = myLora.initOTAA(appEui, appKey,devEui);
  while(!join_result)
  {
    sleep_time(60000); //delay a minute before retry
    join_result = myLora.init();
    Serial.print("FAILED LORA");
  }
  Serial.println("JOINED");
}

//LoRa power uitschakelen
void lora_poweroff(){
  digitalWrite(LoRaEnable, LOW);
}

//LoRa temperatuur versturen
//0.1 °C Signed MSB
void transmit_temp(float temp)
{
  int d_channel = 1;
  int d_type = 103;
  int d_temp = int(temp*10);
  char d_send[16] = {0};
  sprintf(d_send,"%02X%02X%04X",d_channel,d_type,d_temp);
  myLora.txCommand("mac tx uncnf 1 ", d_send, false);
}

//LoRa GPS locatie versturen
//Latitude : 0.0001 ° Signed MSB
//Longitude : 0.0001 ° Signed MSB
//Altitude : 0.01 meter Signed MSB
void transmit_location(double latitude, double longitude, double altitude)
{
  int d_channel = 1;
  int d_type = 136;
  long d_lat = long(latitude*10000);
  long d_lon = long(longitude*10000);
  long d_alt = long(altitude*100);
  if(latitude < 0)
    d_lat = d_lat & 16777215;
  if(longitude < 0)
    d_lon = d_lon & 16777215;
  if(altitude < 0)
    d_alt = d_alt & 16777215;
  char d_send[30] = {0};
  sprintf(d_send,"%02X%02X%06lX%06lX%06lX",d_channel,d_type,d_lat,d_lon,d_alt);
  Serial.println(d_send);
  myLora.txCommand("mac tx uncnf 1 ", d_send, false);
  Serial.println("einde lora");
}

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
  sleep_time(3);;                                   // Wacht even op het starten van de sensor
  sensor.setMeasurementMode(TEMP_AND_HUMID);  // Lees temp en vochtigheid
  sensor.setRate(ONE_MINS);                   // Genereer een wake-up siganl voor de esp32 om de minuut               
  sensor.setTempRes(FOURTEEN_BIT);            // Set de resolutie van temp meting
  sensor.setHumidRes(FOURTEEN_BIT);           // Set de resolutie van vocht meting
  sensor.setInterruptPolarity(ACTIVE_HIGH);   // Interrupt logica actief high
  sensor.enableInterrupt();                   // Enable de wake-up interrupt
  sensor.enableDRDYInterrupt();               // Geef een interrupt als er nieuwe data beschikbaar is
  sensor.triggerMeasurement();                // Start de sensor

  bootCount++;  // Verhoog de counter na het uitvoeren van de methode. Hierdoor wordt deze maar 1x uitgevoerd
}


//Setup als systeem uit deepsleep komt
void wakeUpRoutine()
{
  temptimer++;
  temperature = sensor.readTemp();    // Lees de temperatuur
  humidity = sensor.readHumidity();   // Lees de vochtigheid
  if(temptimer==MINUTES_SEND){
    init_rn2483();
    transmit_temp(temperature);
    lora_poweroff();
    temptimer=0;
  }
  Serial.print("TEMP(C): ");
  Serial.print(temperature);
  Serial.print(" & RH: ");
  Serial.println(humidity);
}

void initGPS() 
{
  GPS.begin(GPS_DEFAULT_I2C_ADDR);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //GPS mode: RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  sleep_time(1000);

  // Ask for firmware version
  GPS.println(PMTK_Q_RELEASE);
  isInitiatedGPS = true;
}

void readGPS()
{
  char c = GPS.read();
  if (GPSECHO);
    //if (c) Serial.println(c);
  if (GPS.newNMEAreceived()) { //nieuwe NMEA command gekregen
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
	//parse gekregen command
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // approximately every 1 seconds or so, print out the current stats
  sleep_time(1000);
  Serial.println("IN TIMER");
  //timer = millis(); // reset the timer
  if (GPS.fix) {
    Serial.println("in de gps fix if");
    gpsLat = GPS.latitude; gpsLatNS = GPS.lat; // latitude: llll.ll, N/S: a
    gpsLon = GPS.longitude; gpsLonEW = GPS.lon; // longitude: yyyyy.yy, E/W: a
    gpsAlt = GPS.altitude; // x.x
    gotLocation = true;
    // printen voor de debug
    if (GPSECHO)
    {
	    Serial.println("Location: ");
	    Serial.print("Latitude");Serial.println(gpsLat);  
	    Serial.print("Longitude");Serial.println(gpsLon);
	    Serial.print("Altitude: "); Serial.println(gpsAlt);
    }
  }
}

//Methode die de hallsensor uitleest
void readHallSensor(){
  uint16_t hallDrempelL = 200;
  uint16_t hallDrempelH = 2400;
  
  pinMode(hallDisablePin, OUTPUT);        // Pin als output zetten
  gpio_hold_dis(hallDisablePin);          // Disable hold functie -> sensor komt in active mode
  int hallData = analogRead(hallOutPin);  // Lees analoge waarde van sensor
  digitalWrite(hallDisablePin, HIGH);     // Zet de enable pin van de sensor hoog -> sensor komt in slaap mode
  gpio_deep_sleep_hold_en();              // Enable de hold functie tijdens deepsleep
  gpio_hold_en(hallDisablePin);           // Enable de hold functie voor bepaalde pin -> voorkomt dat pin laag komt als de esp slaapt!!

  //Serial.println(hallData);

  //Threshold logica
  //DEUR STAAT "OPEN" ALS MAGNEET NAAST DE SENSOR is
  //Puur voor te testen!!
  if (hallData <= hallDrempelL || hallData >=hallDrempelH){
    Serial.println("Deur open!!");
    if (!isInitiatedGPS){
      digitalWrite(GpsEnable,HIGH);
      initGPS();
    }
    int waiting = 0;
    sleep_time(10000); // wacht op fix
    while (!gotLocation && waiting < 60){ //het stopt met proberen na 5 minuten, stuurt alleen om de seconde
      readGPS(); //delay van 1 seconde inbegrepen
      waiting ++;
      Serial.println(waiting);
    }
    init_rn2483();
    if (waiting >= 60){
      //stuur GPSERROR via LoRa
	    transmit_location(0.0,0.0,0.0);
      if (GPSECHO)
        Serial.println("GPS Error");
    }
	  if (gotLocation)
	  {
		  transmit_location((double) gpsLat, (double) gpsLon, (double) gpsAlt);
	  }
    lora_poweroff();
  }
}

//Main setup
void setup()
{
  pinMode(GpsEnable,OUTPUT);
  Serial.begin(115200);
  while (!Serial);
  Serial2.begin(57600);
  while (!Serial2);
  sensor.begin();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_16, 0);
  //Logica die kijkt indien het systeem uit slaap komt of als het voor het eerst start
  if (bootCount == 0)
    firstBoot();
  else
    wakeUpRoutine();

  readHallSensor();
  
  //zet GPS uit
  digitalWrite(GpsEnable,LOW);
  isInitiatedGPS = false;
  gotLocation = false;


  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1);   //Selecteer wake-up pin
  Serial.println("Going to sleep now");
  //Slaap
  esp_deep_sleep_start();
}

void loop()
{
  //Never in loop!!
}
