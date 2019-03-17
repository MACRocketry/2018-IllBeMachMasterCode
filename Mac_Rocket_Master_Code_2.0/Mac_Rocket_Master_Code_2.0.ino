//Start of Import libraries ==============================================================================
//BMP library --------------------
#include <Wire.h>                 //Wire library needed for I2C communication
#include <MacRocketry_BMP_180.h>  //Sparkfun modified library for BMP180

//GPS library --------------------
//#define Include_GPS //comment out this line to NOT use GPS
#ifdef Include_GPS
#include <MacRocketry_GPS_Shield.h>     //Jerry's library for the GPS shield
#endif

//SD Card library --------------------
#include <SPI.h>  //native library for serial peripheral interfacing (sd shield)
#include <SD.h>   //native library for SD reading nd writing
#include <MacRocketry_SD_Logger.h>      //SD Logger library

//LED Indicator library --------------------
#include <MacRocketry_LED_Indicator.h>  //Status indicator library

//End of Import libraries ================================================================================
//Start of #define =======================================================================================
//Data Process constant ----------------------------------------
#define Linear_Interpolate //comment out to not use linearInterpolate
#define sensorAltWeight (0.6)

//Velocity constant
#define velTakeOff (0.005) //meter per millisecond
//#define velDecentDrogue (-0.005) //meter per millisecond

//Timer constant
#define minTimeToApogee_K (24000) //millisecond = 24 seconds from preflight to apogee
#define minTimeToDrogue_K (2000) //milliseconds = 2 seconds from apogee to drogue
#define minTimeToMain_K (10000) //milliseconds = 10 seconds from drogue to main

//Altitude constant
#define altMainMax_K (450) //meter which main is allowed to deploy

//Chute IO Pins ==============================
#define drogueChutePin 3
#define mainChutePin 4

//End of #define =========================================================================================
//Start of Initialize Sensor Objects =====================================================================
#ifdef Include_GPS
MacRocketry_GPS_Shield gps;
#endif

MacRocketry_BMP_180 bmp;
MacRocketry_SD_Logger sd;
MacRocketry_LED_Indicator led;

float alt[2], vel[2]; //acc[1];
float altGndLevel, altMainMax;

uint32_t t[2];
uint32_t timeZero, minTimeToApogee, minTimeToDrogue, minTimeToMain;


//Start of Data Acquisition functions --------------------
void readSensorData(void){
  #ifdef Include_GPS
  if (gps.readSerialBuffer()){
    sd.writeBuffer(gps.getData());
  }
  #endif
  
  if (bmp.readData()){
    sd.writeBuffer( //log BMP data
      "$BMP,temp," + String(bmp.getTemperature()) +
      ",press," + String(bmp.getPressure()) +
      ",alt," + String(bmp.getAltitude()) +
      "\n");
    
    //read time
    t[1] = t[0];
    t[0] = millis();
    
    //read altitude
    alt[1] = alt[0];
    
    //linear interpolate prediction
    #ifdef Linear_Interpolate
    float dT = (float)(t[0] - t[1]);
    float altPredict = alt[0] + vel[0]*dT;
    alt[0] = ( sensorAltWeight*bmp.getAltitude() ) + ( (1.0-sensorAltWeight)*altPredict );
    
    #else // no linear interpolate
    alt[0] = bmp.getAltitude();
    #endif
    
    //read velocity
    vel[1] = vel[0];
    vel[0] = (alt[0] - alt[1]) / dT;
    
    sd.writeBuffer( //log UNO calculated data
      "$UNO,t," + String(t[0]) +
      "alt," + String(alt[0]) +
      "vel," + String(vel[0]) +
      "\n");
      
  }
}

//End of Data Acquisition functions --------------------
//End of Initialize Sensor Objects =======================================================================
//Start of Rocket State Machine ==========================================================================
//Rocket State Machine enum and variables --------------------
enum RocketState {
  Init,
  Preflight,
  Flight,
  DrogueChute,
  MainChute,
  Landed
};
RocketState currentState = Init;

//Start of Rocket State Machine functions --------------------
void nextRocketState(RocketState state); //Arduino IDE bugs out when passing enum parameter
void nextRocketState(RocketState state){
  if (currentState == state){ //extra redundancy
    switch (currentState){
      case Init:
        sd.writeBuffer("@Preflight\n");
        led.setRGB(0, 255, 0);
        delay(1500);
        currentState = Preflight;
        break;
      case Preflight:
        sd.writeBuffer("@Flight\n");
        led.setRGB(127, 255, 0);
        currentState = Flight;
        break;
      case Flight:
        sd.writeBuffer("@DrogueChute\n");
        led.setRGB(255, 140, 0);
        currentState = DrogueChute;
        break;
      case DrogueChute:
        sd.writeBuffer("@MainChute\n");
        led.setRGB(255, 165, 0);
        currentState = MainChute;
        break;
      case MainChute:
        sd.writeBuffer("@Landed\n");
        currentState = Landed;
        break;
    }
  }
}

void processRocketState(){
  readSensorData(); //read all sensor data
  
  switch (currentState){ //process state
    case Init:
      rocketInit();
      break;
    case Preflight:
      rocketPreflight();
      break;
    case Flight:
      rocketFlight();
      break;
    case DrogueChute:
      rocketDrogueChute();
      break;
    case MainChute:
      rocketMainChute();
      break;
    case Landed:
      rocketLanded();
      break;
  }
}
//End of Rocket State Machine functions --------------------
//Start of Rocket State-Specific functions --------------------
void rocketInit(){ //initialize all variables to zero
  led.setRGB(255, 0, 0);
  delay(1500);
  
  for (int i = 0; i < sizeof(t)/sizeof(*t); i++) t[i] = 0;
  for (int i = 0; i < sizeof(alt)/sizeof(*alt); i++) alt[i] = 0;
  for (int i = 0; i < sizeof(vel)/sizeof(*vel); i++) vel[i] = 0;
  #if acc //if acc exist
  for (int i = 0; i < sizeof(acc)/sizeof(*acc); i++) acc[i] = 0;
  #endif
  
  #ifdef Include_GPS
  led.setStatusGPS(gps.getFix());
  #endif
  led.setStatusBMP(bmp.getConnectBMP());
  led.setStatusSD(sd.getConnectFile());
  
  if (!bmp.getConnectBMP()) while (1);  //stop rocket
  if (!sd.getConnectFile()) while (1);  //stop rocket
  
  for (int i = 0; i < 10; i++) readSensorData(); //read in first 10 data points
  altGndLevel = (alt[0] + alt[1]) / 2; //get average Gnd altitude
  
  nextRocketState(Init);
}

void rocketPreflight(){
  altGndLevel = ( altGndLevel*0.8 ) + ( alt[1]*0.2 ); //weighted average altitude
  sd.writeBuffer( //log UNO calculated data
      "$Pre,altGND," + String(altGndLevel) +
      "\n");
  
  if ( //if all prev and curr vel is greater than velTakeOff
    (velTakeOff <= vel[1]) &&
    (velTakeOff <= vel[0])
  ){ //then rocket is in flight
    minTimeToApogee =  millis() + minTimeToApogee_K; //set ~24s to reach apogee
    altMainMax = altGndLevel + altMainMax_K; //set IREC 450m
    
    sd.writeBuffer( //log UNO calculated data
      "$Pre,tApogee," + String(minTimeToApogee) +
      "\n");
    
    nextRocketState(Preflight);
  }
}

void rocketFlight(){
  //ONLY check if apogee timer is reached and altitude decrease
  //does NOT detonate Drogue Chute
  if (
    (minTimeToApogee < millis()) &&   //if minTimeToApogee elapsed
    ((alt[0] - alt[1]) <= 0)          //altitude decrease, apogee reached
  ){ //then reached apogee
    minTimeToDrogue =  millis() + minTimeToDrogue_K; //set ~2s to deploy Drogue

    sd.writeBuffer( //log UNO calculated data
      "$Flight,tDrogue," + String(minTimeToDrogue) +
      "\n");
    
    nextRocketState(Flight);
  }
}

void rocketDrogueChute(){
  if ( //wait for ~2 seconds, then detonate DrogueChute
    (minTimeToDrogue < millis())  //if minTimeToDrogue elapsed
  ){
    minTimeToMain =  millis() + minTimeToMain_K; //set ~10s to deploy Main
    
    sd.writeBuffer( //log UNO calculated data
      "$Drogue,tMain," + String(minTimeToDrogue) +
      "\n");
    
    digitalWrite(drogueChutePin, HIGH); //detonate Drogue
    nextRocketState(DrogueChute);
  }
}

void rocketMainChute(){
  if ( //wait for ~10 seconds and altitude is below IREC 450m
    (minTimeToMain < millis()) &&   //if minTimeToMain elapsed
    (alt[1] <= altMainMax) &&        //if both altitude is below IREC 405m
    (alt[0] <= altMainMax)
  ){ //then detonate Main Chute
    sd.writeBuffer( //log UNO calculated data
      "$Main," + String(millis()) +
      "\n");
    
    digitalWrite(mainChutePin, HIGH);
    nextRocketState(MainChute);
  }
}

void rocketLanded(){
  if ( //if altitude change is within 3m
    ((alt[0] - alt[1]) <= 3)
  ){
    sd.writeBuffer( //log UNO calculated data
      "$Landed\n");
  }
}

//End of Rocket State-Specific functions --------------------
//End of Rocket State Machine ============================================================================
//Start of setup() and loop() ============================================================================
void setup(){ //redundancy initialize all variables to safety

  //set all motor driver pin to low
  pinMode(drogueChutePin, OUTPUT);
  pinMode(mainChutePin, OUTPUT);
  digitalWrite(drogueChutePin, LOW);
  digitalWrite(mainChutePin, LOW);

  //set minTimeToAll to very large value for safety
  timeZero = 3600000; //set min time to 1 hour
  minTimeToApogee = 3600000; //set min time to 1 hour
  minTimeToDrogue = 3600000; //set min time to 1 hour
  minTimeToMain = 3600000; //set min time to 1 hour

  //set altMainMax to very low for safety
  altMainMax = 0;
  
  currentState = Init; //reset current state to Init

}

void loop(){
  processRocketState();
}


//End of setup() and loop() ==============================================================================
