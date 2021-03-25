#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Wire.h>
#include <GPS.h>
#include <BNO055.h>
#include <IMUMaths/imumaths.h>
#include <MatrixMath.h>

//Some defines
#define USE_SERIAL true       //Set to false for release
#define WAIT_FOR_GPS false    //Set to false only for testing
#define USE_DATALOGGER true   //
#define LOG_TO_SD_CARD true   //
#define USE_GPS false         //Set to false only for testing
#define USE_IMU true          //Set to false only for testing
#define TEST_LOOP_TIMEOUT 100 //[ms]
#define GPSSerial Serial2     //What hardware serial port does the GPS use? (Check pinout)

//Variables
bool ledPin = 13;
uint8_t state = 0;              //See the state & error spreadsheet
uint8_t errorCode = 0;          //See the state & error spreadsheet
uint8_t gpsMissedMessage = 0;   //Counter for each time the main loop doesn't have a new GPS message
unsigned long testLoopTimer = 0;    //

struct SensorData {   //Structure with all sensor data to be used
  float pitch;        //Pitch angle in radians
  float yaw;          //Yaw angle in radians
  float roll;         //Roll angle in radians
  float heading;      //Compass heading of nose in radians
  float track;        //Direction the boat is travelling in in radians
  float latitude;     //Latitude in radians
  float longitude;    //Longitude in radians
  float speed;        //Speed in m/s
  float temperature;  //On board temperature measured by the IMU in °C
  float battVoltage;  //Battery voltage
  uint8_t hour;       //Clock hour from GPS
  uint8_t minute;     //Clock minute from GPS
  uint8_t second;     //Clock second from GPS

  //TEMPORARY FOR TESTING:
  float oriX;
  float oriY;
  float oriZ;
} sensorData;

//Forward declarations
bool getSensorData();
bool startup();
void printSensorData();

//Create sensor objects
Adafruit_GPS GPS(&GPSSerial);
Adafruit_BNO055 BNO = Adafruit_BNO055(55, 0x28);

//Let's get to setup!
void setup() {
  delay(250);
}

void loop() {
  if(testLoopTimer > millis()) testLoopTimer = millis();

  switch(state) {
    case 0:
      if(!startup()) {
        errorCode = 1;
        state = 255;
      }
    break;
    case 254:
      if(millis() - testLoopTimer >= TEST_LOOP_TIMEOUT) {
        testLoopTimer = millis();
        if(!getSensorData()) state = 255;
        if(USE_SERIAL) printSensorData();
      }
    break;
    case 255:
      Serial.print("ERROR "); Serial.println(errorCode);
    break;
  }
}

void printSensorData() {
  Serial.println(sensorData.oriX, 3);
  Serial.println(sensorData.oriY, 3);
  Serial.println(sensorData.oriZ, 3);
  Serial.println("---");
}

bool startup() {
  if(USE_SERIAL) Serial.begin(115200);

  if(!GPS.begin(9600)) {
    if(USE_SERIAL) Serial.println(F("Error initializing GPS, check wiring and restart!"));
    return false;
  }
  if(!BNO.begin()) {
    if(USE_SERIAL) Serial.println(F("Error initializing IMU, check wiring or I2C address and restart!"));
    return false;
  }

  //GPS Initialization
  delay(100);
  GPS.sendCommand(PMTK_SET_BAUD_115200); //set baud rate to 115200 bps to allow more data transfer
  delay(100);
  GPSSerial.end(); //close the 9600 baud gps serial
  delay(100);
  GPS.begin(115200); //begin a new serial at 115200 bps
  delay(100);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGAGSA); //Retreive more data from the GPS, only used in testing
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_ENABLE_SBAS);
  GPS.sendCommand(PMTK_ENABLE_EASY);
  GPS.sendCommand(PMTK_ENABLE_AIC);

  //IMU Initialization
  BNO.setExtCrystalUse(true);

  if(WAIT_FOR_GPS) //Do we wait for GPS fix?
  {
    if(USE_SERIAL) Serial.println(F("Waiting for GPS fix..."));
    //If there's no fix,
    //OR the satellites in use are fewer than 8,
    //We just wait 10 seconds and check again
    //These tests should mean that once the loop starts running, we already have decent GPS signal quality

    while(GPS.fix < 1 || (int)GPS.satellites <= 8) {
      Serial.print(F("No. of satellites: ")); Serial.println((int)GPS.satellites);
      Serial.println(F("No GPS fix found, checking again in 10 seconds..."));
      delay(10000);
    }
    if(USE_SERIAL) Serial.println(F("GPS fix found!"));
  } else {
    if(USE_SERIAL) Serial.println(F("Skipping GPS fix!"));
  }
  state = 254;
  return true;
}

bool getSensorData() {
  //code for parsing GPS data
  if(USE_GPS) {
    if(GPS.newNMEAreceived()) {
      gpsMissedMessage = 0;
      if(!GPS.parse(GPS.lastNMEA())) {
        if(USE_SERIAL) Serial.println(F("Could not parse NMEA sentence!"));
        errorCode = 2;
        return false;
      }
    } else {
      gpsMissedMessage++;
      /*
      If we haven't received a message from the GPS for the last 100 loop runs (approx 10 seconds)
      We assume we lost communications and throw an error
      */
      if(gpsMissedMessage >= 100) {
        errorCode = 2;
        return false;
      }
    }
  }

  sensors_event_t IMUevent;
  BNO.getEvent(&IMUevent, Adafruit_BNO055::VECTOR_EULER);
  //For some reason x and z are flipped, we flip them back here
  sensorData.oriX = IMUevent.orientation.x;
  sensorData.oriY = IMUevent.orientation.y;
  sensorData.oriZ = IMUevent.orientation.z;
  sensorData.heading = IMUevent.orientation.heading;

  sensorData.hour = GPS.hour;
  sensorData.minute = GPS.minute;
  sensorData.second = GPS.seconds;
  sensorData.track = GPS.angle;
  sensorData.speed = GPS.speed * (1.852/3.6); //Convert to m/s

  return true;
}