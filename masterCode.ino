//Import libraries here
#include <SPI.h> //native library for serial peripheral interfacing (sd shield)
#include <SD.h> //native library for SD reading nd writing
#include <MacRocketry_GPS_Shield.h>  //Jerrys library for the GPS shield
#include <LED_Diagnostics.h>  //Sttus indicator library
#include <MacRocketry_SD_Logger.h> //SD Logger library
#include <Wire.h>  //OneWire library and Adafruit libraries for the BMP085 sensor.
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

//Initialzie objects here
MacRocketry_SD_Logger logger;  
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(12345);
MacRocketry_GPS_Shield gps;
LED_Diagnostics led;

// In this section are constants used to determine flight stages
#define MINh	5
#define MAINh	8900

#define MINt	60
#define DROt	5
#define MAXt	600

#define NEGv	-5
#define MINv	5 
// In this section are constants used to determine flight stages

enum rocket_state { preflight, flight, drogue, main, landed };
enum rocket_state current_state;

current_state = preflight;

void setup() {
  Serial.begin(115200); //Begin serial transmission for debugging
  Serial.println("Serial transmission successful");
  
  initializeSensors();
  initializeSD();
  
  state = 1;
  stateMachine(state);
}
//This is currently placeholder for the data reading object
float diagnostics(float rocketInfo) {  //this phase will operate before we launch the rocket
  gpsData = readGPS();  //use Jerrys library
  bmpData = readBMP();  //use adafruit library
  rocketInfo = dataAnalytics(gpsData, bmpData);  //returns array containing p, p', v, v', a, a', along with any GPS data we want
  storeData(rocketInfo);  //This funct
  
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

rocket_state state_machine(float flight_altitude, float flight_velocity, float flight_time) {
	
	switch (current_state) {
		case preflight:
      led.statusCheck(*Check to see if we can pass already taken data*);
			if (flight_altitude > MINh && flight_velocity > MINv)
				current_state = flight;
			break;
		case flight:
			if (flight_velocity < NEGv && flight_time > MINt)
				current_state = drogue;
			break;
    case drogue:
      //DEPLOY DROGUE
			if (flight_altitude < MAINh)
				current_state = main;
			break;
		case main:
      //DEPLOY MAIN
			if (flight_altitude < MINh && flight_time > MINv)
				current_state = landed;
			break;
    case landed:
      //STOP LOGGING DATA
      break;
	}
	return current_state
}
