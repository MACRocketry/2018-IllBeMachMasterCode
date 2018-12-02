//Import libraries here
#include <SPI.h> //native library for serial peripheral interfacing (sd shield)
#include <SD.h> //native library for SD reading nd writing
#include <MacRocketry_GPS_Shield.h>  //Jerrys library for the GPS shield
#include <LED_Diagnostics.h>  //Sttus indicator library
#include <MacRocketry_SD_Logger.h> //SD Logger library
#include <Wire.h>  //OneWire library and Adafruit libraries for the BMP085 sensor.
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

//Jay is the best!

//Initialzie objects here
MacRocketry_SD_Logger logger;  
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(12345);
MacRocketry_GPS_Shield gps;
LED_Diagnostics led;

void setup() {
  Serial.begin(115200); //Begin serial transmission for debugging
  Serial.println("Serial transmission successful");
  
  initializeSensors();
  initializeSD();
  
  state = 1;
  stateMachine(state);
}

void stateMachine(int state) {
  rocketInfo = [float array]; //static memory array initialized
  while (True) {
    if (state == 1) {  //diagnostics stage
      rocketInfo = diagnostics(rocketInfo);
      if (condition for next state) {
        state = 2;
      }
    }
    
    else if (state == 2) {  //ascentToDrogue stage
      rocketInfo = ascentToDrogue(rocketInfo);
      if (condition for next state) {
        state = 3;
      }
    }
    
    else if (state == 3) {  //descentToMain
      rocketInfo = descentToMain(rocketInfo);
      if (condition for next state) {
        state = 4;
      }
    }
    
    else if (state == 4) {  //descentToGround
      rocketInfo = descentToGround(rocketInfo);
    }
  }
}

float diagnostics(float rocketInfo) {  //this phase will operate before we launch the rocket
  gpsData = readGPS();  //use Jerrys library
  bmpData = readBMP();  //use adafruit library
  rocketInfo = dataAnalytics(gpsData, bmpData);  //returns array containing p, p', v, v', a, a', along with any GPS data we want
  storeData(rocketInfo);  //This funct
  
  led.statusCheck(*Check to see if we can pass already taken data*);
  
  return rocketInfo;
}

float ascentToDrogue(float rocketInfo) {  //this phase will run up to and including laucnhing the drogue chute
  gpsData = readGPS();  //use Jerrys library
  bmpData = readBMP();  //use adafruit library
  rocketInfo = dataAnalytics(gpsData, bmpData);  //returns array containing p, p', v, v', a, a', along with any GPS data we want
  storeData(rocketInfo);
  
  return rocketInfo;
}

void descentToMain(float rocketInfo) {  //this phase will operate starting after the deployment of the drogue up to and including the deployment of the main chute
  gpsData = readGPS();  //use Jerrys library
  bmpData = readBMP();  //use adafruit library
  rocketInfo = dataAnalytics(gpsData, bmpData);  //returns array containing p, p', v, v', a, a', along with any GPS data we want
  storeData(rocketInfo);
  
  return rocketInfo;
}

void descentToGround(float rocketInfo) {  //this phase ill run after the chutes have been deployed and until the rocket lands
  gpsData = readGPS();  //use Jerrys library
  bmpData = readBMP();  //use adafruit library
  rocketInfo = dataAnalytics(gpsData, bmpData);  //returns array containing p, p', v, v', a, a', along with any GPS data we want
  storeData(rocketInfo);
  
  return rocketInfo;
}

bool initializeSD() {
  //create a file and open
  //prep the SD card so that we do not have to continously open and close files
  //this should only run once
  logger = MacRocketry_SD_Logger();
  if(logger.connectFile == NULL)
  {
    Serial.println("Failed to open log file!");
    return false;
  }
}

//Function to be called once at the start to initialize any onboard sensors
//Returns boolean false if any initializations fail. (SUBJECT TO CHANGE)
bool initializeSensors() {
  bool success = true;
  //Begin BMP sensor
  if (!bmp.begin())
    success = false;
  return success;
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

//Grabs pressure data from the BMP sensor
float readBMP()
{
  float pressure = 0;
  bmp.getPressure(&pressure);
  return pressure;
}
