//Start of Import libraries ==============================================================================
//BMP library --------------------
#include <Wire.h>               //Wire library needed for I2C communication.
#include <Adafruit_Sensor.h>    //Adafruit sensor library needed for BMP library
#include <Adafruit_BMP085_U.h>  //BMP180 and BMP085 use the same library

//GPS library --------------------
#include <MacRocketry_GPS_Shield.h>     //Jerry's library for the GPS shield

//SD Card library --------------------
#include <SPI.h>  //native library for serial peripheral interfacing (sd shield)
#include <SD.h>   //native library for SD reading nd writing
#include <MacRocketry_SD_Logger.h>      //SD Logger library

//LED Indicator library --------------------
#include <MacRocketry_LED_Indicator.h>  //Status indicator library

//End of Import libraries ================================================================================
//Start of Initialize Sensor Objects =====================================================================
Adafruit_BMP085_Unified bmp;
MacRocketry_GPS_Shield gps;
MacRocketry_SD_Logger sd;
MacRocketry_LED_Indicator led;

//End of Initialize Sensor Objects =======================================================================
//Start of Rocket State Machine ==========================================================================
//Rocket State Machine enum and variables --------------------
enum RocketState {
  Init,
  Preflight,
  Flight,
  DrogueShoot,
  MainShoot,
  Landed
};
RocketState currentState = Init;

//Start of Rocket State Machine functions --------------------
void nextRocketState(bool safeToTransition){
  if (safeToTransition){
    switch (currentState){
      case Init:
        currentState = Preflight;
        break;
      case Preflight:
        currentState = Flight;
        break;
      case Flight:
        currentState = DrogueShoot;
        break;
      case DrogueShoot:
        currentState = MainShoot;
        break;
      case MainShoot:
        currentState = Landed;
        break;
    }
  }
}

void processRocketState(){
  switch (currentState){
    case Init:
      rocketInit();
      break;
    case Preflight:
      rocketPreflight();
      break;
    case Flight:
      rocketFlight();
      break;
    case DrogueShoot:
      rocketDrogueShoot();
      break;
    case MainShoot:
      rocketMainShoot();
      break;
    case Landed:
      rocketLanded();
      break;
  }
}
//End of Rocket State Machine functions --------------------
//Start of Rocket State-Specific functions --------------------
void rocketInit(){
  
}

void rocketPreflight(){
  
}

void rocketFlight(){
  
}

void rocketDrogueShoot(){
  
}

void rocketMainShoot(){
  
}

void rocketLanded(){
  
}

//End of Rocket State-Specific functions --------------------
//End of Rocket State Machine ============================================================================

//setup() and loop() ================================================================================
void setup(){
  
}

void loop(){
  processRocketState();
}

/*
//Start of Setup() =======================================================================================
void setup(){
  Serial.begin(115200); //Begin serial communication for debugging
  Serial.println("Start serial communication...");

}


//Start of Draft Code ====================================================================================
void setup() {
  Serial.begin(115200); //Begin serial communication for debugging
  Serial.println("Start serial communication...");
  
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

*/
