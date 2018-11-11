#include <SPI.h> //native library for serial peripheral interfacing (sd shield)
#include <SD.h> //native library for SD reading nd writing
#include <MacRocketry_GPS_Shield.h>  //Jerrys library for the GPS shield
#include <LEDDiagnostics.h>  //Sttus indicator library

MacRocketry_GPS_Shield gps;
LED_Diagnostics led;

void setup() {
  Serial.begin(115200); //Begin serial transmission for debugging
  Serial.println("Serial transmission successful");
  initializeSD();
  rocketInfo = [float array]; //static memory array initialized
  diagnostics(rocketInfo);
}

float diagnostics(float rocketInfo) {  //this phase will operate before we launch the rocket
  gpsData = readGPS();  //use Jerrys library
  bmpData = readBMP();  //use adafruit library
  rocketInfo = dataAnalytics(gpsData, bmpData);  //returns array containing p, p', v, v', a, a', along with any GPS data we want
  storeData(rocketInfo);  //This funct
  
  led.statusCheck(*Check to see if we can pass already taken data*);
  
  if (conditionToNextPhase) {
    ascentToDrogue(rocketInfo);
  }
  else {
    diagnostics(rocketInfo);
  }
}

float ascentToDrogue(float rocketInfo) {  //this phase will run up to and including laucnhing the drogue chute
  gpsData = readGPS();  //use Jerrys library
  bmpData = readBMP();  //use adafruit library
  rocketInfo = dataAnalytics(gpsData, bmpData);  //returns array containing p, p', v, v', a, a', along with any GPS data we want
  storeData(rocketInfo);
  
  if (conditionToNextPhase) {
    deployDrogue();
    descentToMain(rocketInfo);
  }
  else {
    ascentToDrogue(rocketInfo);
  }
}

void descentToMain(float rocketInfo) {  //this phase will operate starting after the deployment of the drogue up to and including the deployment of the main chute
  gpsData = readGPS();  //use Jerrys library
  bmpData = readBMP();  //use adafruit library
  rocketInfo = dataAnalytics(gpsData, bmpData);  //returns array containing p, p', v, v', a, a', along with any GPS data we want
  storeData(rocketInfo);
  
  if (conditionToNextPhase) {
    deployMain();
    descent(rocketInfo);
  }
  else {
    descentToMain(rocketInfo);
  }
}

void descent(float rocketInfo) {  //this phase ill run after the chutes have been deployed and until the rocket lands
  gpsData = readGPS();  //use Jerrys library
  bmpData = readBMP();  //use adafruit library
  rocketInfo = dataAnalytics(gpsData, bmpData);  //returns array containing p, p', v, v', a, a', along with any GPS data we want
  storeData(rocketInfo);
}

void initializeSD() {
  //create a file and open
  //prep the SD card so that we do not have to continously open and close files
  //this should only run once
}

float gpsReadAndWrite() {
  if (gps.readData()){
    Serial.print(gps.data); //access NMEA data
    Serial.print("UTC: ");
    Serial.print(gps.utc); //access UTC [float]
    Serial.print(" Fix: ");
    Serial.print(gps.fix); //acess fix [int]
    Serial.print(" Altitude ");
    Serial.println(gps.altitude); //acess altitude [float]
  }
}
