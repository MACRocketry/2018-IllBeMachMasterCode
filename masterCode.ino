#include <SPI.h> //native library for serial peripheral interfacing (sd shield)
#include <SD.h> //native library for SD reading nd writing
#include <MacRocketry_GPS_Shield.h>  //Jerrys library for the GPS shield

MacRocketry_GPS_Shield gps;

void setup() {
  Serial.begin(115200); //Begin serial transmission for debugging
  Serial.println("Serial transmission successful");
  initializeSD();
  diagnostics();
}

float diagnostics() {  //this phase will operate before we launch the rocket
  gpsAltitude = readGPS();
  diagnostics();
}

void ascentToDrogue() {  //this phase will run up to and including laucnhing the drogue chute
  
}

void descentToMain() {  //this phase will operate starting after the deployment of the drogue up to and including the deployment of the main chute
  
}

void descent() {  //this phase ill run after the chutes have been deployed and until the rocket lands
  
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
