//Rocket team code by Jeremey Waldron
//Measures gyroscopic, atmospheric, and temperature data and then saves it on an SD card. 
/*
  Pin Connections:

  SD Card:
  CS- 4
  DI- 51
  DO- 50
  CLK- 52
  3V- 3.3V
  GND- GND

  GYRO:
  SCL- SCL
  SDA- SDA
  5V- 5V
  GND- GND

  GPS:
  5V- 5V
  GND- GND
  YELLOW- 2
 */


//SD card libraries
#include <SPI.h>
#include <SD.h>

//gyroscope libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

//gps libraries
#include <SoftwareSerial.h>
//#include <TinyGPS.h>

//GPS variables
//TinyGPS gps;
//#define ss Serial1

//SD card variables
const int chipSelect = 4;

int counter = 0;

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; 

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}

/**************************************************************************/
/*!

*/
/************************
**************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 10 DOF Pitch/Roll/Heading Example")); Serial.println("");
//  ss.begin(4800);
//  Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  
  /* Initialise the sensors */
  initSensors();
    // Open serial communications and wait for port to open:
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}


/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
//create string to save data to
String dataString = "RocketData: ";

  dataString += "Time: (in ms) " + String( millis()) + ";";
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
//    Serial.print(F("Roll: "));
//    Serial.print(orientation.roll);
    dataString += " Roll: " + String(orientation.roll);
//    Serial.print(F("; "));
//    Serial.print(F("Pitch: "));
//    Serial.print(orientation.pitch);
    dataString += "; Pitch: " + String(orientation.pitch);
//    Serial.print(F("; "));
  }
  
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
//    Serial.print(F("Heading: "));
//    Serial.print(orientation.heading);
    dataString += "; Heading: " + String(orientation.heading);
//    Serial.print(F("; "));
  }

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    /* Convert atmospheric pressure, SLP and temp to altitude    */
//    Serial.print(F("Alt: "));
//    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
//                                        bmp_event.pressure,
//                                        temperature)); 
    dataString += "; Altitude: " + String(bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure,temperature));

//    Serial.print(F(" m; "));
    /* Display the temperature */
//    Serial.print(F("Temp: "));
//    Serial.print(temperature);
    dataString += "; Temperature: " + String(temperature) + " C ";
//    Serial.print(F(" C"));
  }

//  //START GPS
//  bool newData = false;
//  unsigned long chars;
//  unsigned short sentences, failed;
//
//  // For one second we parse GPS data and report some key values
//  for (unsigned long start = millis(); millis() - start < 1000;)
//  {
//    while (ss.available())
//    {
//      char c = ss.read();
//      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
//      if (gps.encode(c)) // Did a new valid sentence come in?
//        newData = true;
//    }
//  }
//
////  if (newData)
////  {
//    float flat, flon;
//    unsigned long age;
//    gps.f_get_position(&flat, &flon, &age);
////    Serial.print("LAT=");
////    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
//    dataString += "; LAT: " + String(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
//
////    Serial.print(" LON=");
////    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
//    dataString += "; LON: " + String(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
//
//
////    Serial.print(" SAT=");
////    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
//    dataString += "; SAT: " + String(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
//    dataString += "; GPS Speed(kmph): " + String(String(gps.f_speed_kmph()));
//    dataString += "; GPS Altitude(m): " + String( gps.f_altitude() );
//    
//    
////    Serial.print(" PREC=");
////    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
////  }
//  
////  gps.stats(&chars, &sentences, &failed);
////  Serial.print(" CHARS=");
////  Serial.print(chars);
////  Serial.print(" SENTENCES=");
////  Serial.print(sentences);
////  Serial.print(" CSUM ERR=");
////  Serial.println(failed);
//  if (chars == 0){
//    Serial.println("** No characters received from GPS: check wiring **");
//  }
//  
  Serial.println(F(""));
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  
  
}
