//PINS
//ALTSOFSERIAL RX IS PIN 8


#include <AltSoftSerial.h>
//com 4 is transmitter
#include <chibi.h>
#define DEST_ADDR 5 // this will be address of our receiver
#include <TinyGPS.h>

AltSoftSerial ss(4, 3);
TinyGPS gps;


void setup()
{
  Serial.begin(115200);
  chibiInit();
  chibiSetShortAddr(3);
  ss.begin(4800);

} 


void loop()
{
  //initialize variable to store data to
  String dataString;
  dataString += "Time(sec);" + String( (millis()/1000)) ;

  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

bool newData = false;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

//  if (newData)
//  {
    gps.f_get_position(&flat, &flon, &age);
//    Serial.print("LAT=");
//    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    dataString += ";LAT: " + String(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);

//    Serial.print(" LON=");
//    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    dataString += ";LON: " + String(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);


//    Serial.print(" SAT=");
//    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    dataString += ";SAT: " + String(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    dataString += ";Speed(kmph):" + String(String(gps.f_speed_kmph()));
    dataString += ";Altitude(m):" + String( gps.f_altitude() );

  
  byte dataBuf[100];
  int i = 0;
  for(i=0;i<100;i++){
    dataBuf[i]=dataString[i];
  }
  strcpy((char *)dataBuf, (char *)dataBuf);
  chibiTx(DEST_ADDR, dataBuf, strlen((char *)dataBuf)+1);
  delay(10);
  Serial.println(dataString);
  Serial.println("Message Sent");

}

