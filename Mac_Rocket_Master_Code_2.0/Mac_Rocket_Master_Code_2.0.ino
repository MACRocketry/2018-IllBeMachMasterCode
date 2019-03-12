//Start of Import libraries ==============================================================================
//BMP library --------------------
#include <Wire.h>               //Wire library needed for I2C communication.
#include <Adafruit_Sensor.h>    //Adafruit sensor library needed for BMP library
#include <Adafruit_BMP085_U.h>  //BMP180 and BMP085 use the same library

//GPS library -------------------- (ALL GPS COMPONENTS COMMENTED OUT FOR NOW)
//#include <MacRocketry_GPS_Shield.h>     //Jerry's library for the GPS shield

//SD Card library --------------------
#include <SPI.h>  //native library for serial peripheral interfacing (sd shield)
#include <SD.h>   //native library for SD reading nd writing
#include <MacRocketry_SD_Logger.h>      //SD Logger library

//LED Indicator library --------------------
#include <MacRocketry_LED_Indicator.h>  //Status indicator library

//End of Import libraries ================================================================================
//Definitions
#define seaLevel 1013.25 //hPa

//Start of Initialize Sensor Objects =====================================================================
Adafruit_BMP085_Unified bmp;
//MacRocketry_GPS_Shield gps;
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
    }  }
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
  SDcheck = initializeSD();
  BMPcheck =  initialzizeBMP();
  
  led.setStatusGPS(true);
  
  if (SDcheck && BMPcheck) {
    nextRocketState(true);
  }
}

void rocketPreflight(){
  //Next state variables
  float groundLvl = 100; //Sea level in meters
  
  //Read data from BMP
  float pressure = 0;
  float altitude = 0;
  float temp = 0;
  float time = 0;
  readBMP(&pressure, &altitude, &temp, &time);
  
  //Process and store data
  altitudeProcessed = dataAnalytics(altitude);
  previousAltitude = altitudeProcessed;
  sd.writeBuffer("Pressure: " + String(pressure, 4) + "hPa \n")
  sd.writeBuffer("Altitude: " + String(altitude, 4) + "m \n")
  sd.writeBuffer("Processed Altitude: " + String(altitudeProcessed, 4) + "m \n")
  sd.writeBuffer("Temperature: " + String(temp, 4) + "C \n")
  
  led.setStatusBMP(altitude); //*********JAROD NEEDS TO LOOK*********
  //nextRocketState(altitude > groundLvl && vel >> 0);
}

void rocketFlight(){
  //Next state variables
  float minTimeToApogee = 100; //needs to be confirmed with structural
  
  //Read data from BMP
  float pressure = 0;
  float altitude = 0;
  float temp = 0;
  float time = 0;
  readBMP(&pressure, &altitude, &temp, &time);
  
  //Process and store data
  altitudeProcessed = dataAnalytics(altitude);
  previousAltitude = altitudeProcessed;
  sd.writeBuffer("Pressure: " + String(pressure, 4) + "hPa \n")
  sd.writeBuffer("Altitude: " + String(altitude, 4) + "m \n")
  sd.writeBuffer("Processed Altitude: " + String(altitudeProcessed, 4) + "m \n")
  sd.writeBuffer("Temperature: " + String(temp, 4) + "C \n")
  
  //nextRocketState(flight_velocity < 0 && flight_time > minTimeToApogee);
}

void rocketDrogueShoot(){
  //Next state variables
  float minDrogueAltitude = 9500; //needs to be confirmed with structural
  float minTimeApogeeToDrogue = 5; //needs to be confirmed with structural
  
  //Read data from BMP
  float pressure = 0;
  float altitude = 0;
  float temp = 0;
  float time = 0;
  readBMP(&pressure, &altitude, &temp, &time);
  
  //Process and store data
  altitudeProcessed = dataAnalytics(altitude);
  previousAltitude = altitudeProcessed;
  sd.writeBuffer("Pressure: " + String(pressure, 4) + "hPa \n")
  sd.writeBuffer("Altitude: " + String(altitude, 4) + "m \n")
  sd.writeBuffer("Processed Altitude: " + String(altitudeProcessed, 4) + "m \n")
  sd.writeBuffer("Temperature: " + String(temp, 4) + "C \n")
  
  //nextRocketState(flight_altitude < minDrogueAltitude && flight_time > minTimeApogeeToDrogue);
}

void rocketMainShoot(){
  //Next state variables
  float minAltToDeployMain = 2500; //needs to be confirmed with structural
  float minTimeDrogueToMain = 5; //needs to be confirmed with structural
  
  //Read data from BMP
  float pressure = 0;
  float altitude = 0;
  float temp = 0;
  float time = 0;
  readBMP(&pressure, &altitude, &temp, &time);
  
  //Process and store data
  altitudeProcessed = dataAnalytics(altitude);
  previousAltitude = altitudeProcessed;
  sd.writeBuffer("Pressure: " + String(pressure, 4) + "hPa \n");
  sd.writeBuffer("Altitude: " + String(altitude, 4) + "m \n");
  sd.writeBuffer("Processed Altitude: " + String(altitudeProcessed, 4) + "m \n");
  sd.writeBuffer("Temperature: " + String(temp, 4) + "C \n");
  
  //nextRocketState(flight_altitude < minAltToDeployMain && flight_time > minTimeDrogueToMain);
}

void rocketLanded(){
  //Read data from BMP
  float pressure = 0;
  float altitude = 0;
  float temp = 0;
  float time = 0;
  readBMP(&pressure, &altitude, &temp, &time);
  
  //Process and store data
  altitudeProcessed = dataAnalytics(altitude);
  previousAltitude = altitudeProcessed;
  sd.writeBuffer("Pressure: " + String(pressure, 4) + "hPa \n");
  sd.writeBuffer("Altitude: " + String(altitude, 4) + "m \n");
  sd.writeBuffer("Processed Altitude: " + String(altitudeProcessed, 4) + "m \n");
  sd.writeBuffer("Temperature: " + String(temp, 4) + "C \n");
}

//End of Rocket State-Specific functions --------------------
//End of Rocket State Machine ============================================================================



//Sensor functions ===================================================

bool initializeBMP() {
  bool success = true;
  //Begin BMP sensor
  if (!bmp.begin())
    success = false;
  return success;
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
  else{
    return true;
  }
}

void readBMP(float *pressure, float *altitude, float *temp, float *time)
{
  bmp.getTemperature(temp);
  bmp.getPressure(pressure);
  *time = millis();
  *altitude = bmp.pressureToAltitude(seaLevel, pressure/100);
}

//Do we have a better way to store these rather than making them global? (used for dataAnalytics)
float previousAltitude = 0;
float previousVelocity = 0;
float previousTime = 0;
float omega = 0.4;

float dataAnalytics(float altitude, float time) {
  deltaTime = time - previousTime;
  prediction = previousAltitude + deltaTime*previousVelocity;
  
  previousVelocity = (altitude - previousAltitude) / deltaTime;
  
  altitudeProcessed = ((1 - omega) * altitude) + (omega * prediction);
  return altitudeProcessed;
}


//End of senosor functions ==============================================


//setup() and loop() ================================================================================
void setup(){
  
}

void loop(){
  processRocketState();
}


/*  Here lies the conditional statements that need to be ported over

#define MINh    5
#define MAINh    8900

#define MINt    60
#define DROt    5
#define MAXt    600

#define NEGv    -5
#define MINv    5


void main() {
    enum rocket_state {preflight, flight, drogue, main, landed};
    enum rocket_state current_state;
    float flight_altitude = 0;
    float flight_velocity = 0;
    float time = 0;

    current_state = preflight;
    switch (current_state) {
        case preflight:
            if (flight_altitude > MINh && flight_velocity > MINv)
                current_state = flight;
            break;
        case flight:
            if (flight_velocity < NEGv && flight_time > MINt)
                current_state = drogue;
            break;
        case drogue:
            if (flight_altitude < MAINh && flight_time > DROt)
                current_state = main;
            break;
        case main:
            if (flight_altitude < MINh && flight_time > MINv)
                current_state = landed;
            break;
        case landed:
            break;
    }
}
*/

