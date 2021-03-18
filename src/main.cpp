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
#define WAIT_FOR_GPS false
#define USE_DATALOGGER true
#define LOG_TO_SD_CARD true
#define USE_GPS false
#define USE_IMU true
#define LOOP_TIMEOUT 100 // [ms]
#define LEDPIN 13
#define GPSSerial Serial2 //What hardware serial port does the GPS use? (Check pinout)

//Variables
uint8_t state = 255; //Needs documentation, state machine state, 255 is experimental
unsigned long loopTimer = 0;

//Forward declarations
bool getSensorData();

//Create sensor objects
Adafruit_GPS GPS(&GPSSerial);
Adafruit_BNO055 BNO = Adafruit_BNO055(55, 0x28);

void setup() {
  delay(250);
  Serial.begin(115200);
}

void loop() {
  if (loopTimer > millis()) loopTimer = millis();

  if(millis() - loopTimer >= LOOP_TIMEOUT) {
    loopTimer = millis();
    switch (state) {
      case 255:
        //code
      break;
    }
  }
}

/*
  if(!GPS.begin(9600)) {
    Serial.println(F("Error initializing GPS, check wiring and restart!"));
    while(1);
  }
  if(!bno.begin()) {
    Serial.println(F("Error initializing IMU, check wiring or I2C address and restart!"));
    while(1);
  }

//GPS Initialization
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGAGSA);
  delay(100);
  GPS.sendCommand(PMTK_SET_BAUD_57600); //set baud rate to 57600 bps to allow more data transfer
  delay(100);
  GPSSerial.end(); //close the 9600 baud gps serial
  delay(100);
  GPS.begin(57600); //begin a new serial at 57600 bps
  delay(100);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_ENABLE_SBAS);
  GPS.sendCommand(PMTK_ENABLE_EASY);
  GPS.sendCommand(PMTK_ENABLE_AIC);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
  while (GPSSerial.available() > 0) {
    char incomingByte = GPSSerial.read();
    Serial.print(incomingByte);
  }
  Serial.println("");

//IMU Initialization
  bno.setExtCrystalUse(true);

  if(WAIT_FOR_GPS) //Do we wait for GPS fix?
  {
    Serial.println(F("Waiting for GPS fix..."));
    //If there's no fix,
    //OR the satellites in use are fewer than 8,
    //We just wait 10 seconds and check again
    //These tests should mean that once the loop starts running, we already have decent GPS signal quality

    while (GPS.fix != 1 || (int)GPS.satellites <= 8) {
      Serial.print(F("No. of satellites: ")); Serial.println((int)GPS.satellites);
      Serial.println(F("No GPS fix found, checking again in 10 seconds..."));
      delay(10000);
    }
    Serial.println(F("GPS fix found!"));
  } else {
    Serial.println(F("Skipping GPS fix!"));
  }

  //code for parsing GPS
  if(GPS.newNMEAreceived())
  {
    if(!GPS.parse(GPS.lastNMEA()))
    {
      Serial.println(F("Could not parse NMEA sentence!"));
    }
  }

*/