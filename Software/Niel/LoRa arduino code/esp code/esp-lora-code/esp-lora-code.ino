#include <rn2xx3.h>


rn2xx3 myLora(Serial);
uint8_t Reset = D5;


void setup()
{
  Serial.begin(57600); //serial port to LoRa
  delay(10000);
  initialize_rn2483();
  delay(2000);
}

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
  if(latitude < 0){
    d_lat = d_lat & 16777215;
  }
  if(longitude < 0){
    d_lon = d_lon & 16777215;
  }
  if(altitude < 0){
    d_alt = d_alt & 16777215;
  }
  char d_send[22] = {0};
  sprintf(d_send,"%02X%02X%06lX%06lX%06lX",d_channel,d_type,d_lat,d_lon,d_alt);
  myLora.txCommand("mac tx uncnf 1 ", d_send, false);
}
void initialize_rn2483()
{
  //reset rn2483
  pinMode(Reset, OUTPUT);
  digitalWrite(Reset, LOW);
  delay(500);
  digitalWrite(Reset, HIGH);
  delay(100); //wait for the RN2xx3's startup message
  Serial.flush();
  
  myLora.autobaud();

  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    delay(10000);
    hweui = myLora.hweui();
  }
  bool join_result = false;
  const char *appEui = "0000000000000000";
  const char *appKey = "153F68488BE1FE937F4ECA5E330FC4D6";
  const char *devEui = "F801000189040000";
  join_result = myLora.initOTAA(appEui, appKey,devEui);
  while(!join_result)
  {
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
}


void loop()
{
  transmit_temp(25.7);
  delay(5000);
  transmit_location(42.3519,-87.9094,10);
  delay(10000);
}
