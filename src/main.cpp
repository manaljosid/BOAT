#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
/*#include <avr/io.h>
#include <avr/interrupt.h>*/
#include <BNO055.h>
/*#include <IMUMaths/imumaths.h>
#include <BMP280.h>*/

//Some defines
/*
#define WAIT_FOR_GPS false
#define USE_DATALOGGER true
#define LOG_TO_SD_CARD true
#define USE_AIRSPEED false
#define USE_GPS false
#define USE_BAROMETER false
#define USE_IMU true
#define LOOP_LENGTH 100 // [ms]
#define LEDPIN 13
*/
#define LOOP_LENGTH 100 // [ms]
#define LEDPIN 13

//Temporary init code
//#include <GPS.h>
//#define GPSSerial Serial2 //What hardware serial port does the GPS use
//Adafruit_GPS GPS(&GPSSerial);

//variables
//uint8_t state;
unsigned long loopTimer = 0;

//forward declarations
bool getIMUData();
//bool getBaroData();
//bool getGPSData();
//bool dumpFlash();
bool writeDataToSerial(); //Missing pointer to the data itself


//Create sensor objects
//Adafruit_BMP280 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  delay(250);
  Serial.begin(115200);
}

void loop() {
  if (loopTimer > millis()) loopTimer = millis();

  if(millis() - loopTimer >= LOOP_LENGTH) {
    loopTimer = millis();
    /*switch (state) {
      case 255:
        getIMUData();
        writeDataToSerial();
      break;
    }*/
  }
}

bool getIMUData() {
  //code
}

bool writeDataToSerial() {
  //code
}

/*
bool dumpFlash() {
  char fileName[13] = "DLG_0000.CSV"; //Data log file name.
  bool usingDatalogger = false; //Are we using the datalogger?
}
*/