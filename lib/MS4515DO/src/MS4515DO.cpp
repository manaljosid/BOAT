/*
 *  Library for use with the TE Measurements 4515DO-DS3BK030DPL differential pressure sensor.
 *  The sensor communicates over I2C and uses a 3.3V supply.
 *  It is intended to be used as an airspeed sensor onboard model planes.
 *  
 *  Author: Mani Magnusson
 */

#include <MS4515DO.h>
#include <Arduino.h>

MS4515DO::MS4515DO(TwoWire *theWire)
{
  _wire = theWire;
}

/**
 * @brief Start communications with the sensor and zero it.
 *        Note that if the sensor is not in the right condition for zeroing at startup, it requires re-zeroing.
 * 
 * @return True if successful zeroing of sensor, false if unsuccessful.
 */
bool MS4515DO::begin(uint8_t addr)
{
  _addr = (int)addr;
  _wire->begin();
  if(zero()) return true;
  return false;
}

/**
 * @brief Read the last measured pressure from the sensor.
 * 
 * @return The pressure in Pa
 */
float MS4515DO::read_pressure()
{
  return pressure - offsetPressure;
}

/**
 * @brief Read the last measured temperature from the sensor.
 * 
 * @return The temperature in °C
 */
float MS4515DO::read_temperature()
{
  return temperature;
}

/**
 * @brief Fetches a new offset to zero the sensor.
 * 
 * @return True if succsessful, false if retrieving new value is unsuccessful.
 */
bool MS4515DO::zero()
{
  float avgPressure = 0.0;
  for(int i = 0; i < 10; i++)
  {
    if(poll_sensor() != 0) return false;
    avgPressure += pressure;
  }
  avgPressure /= 10;
  offsetPressure = avgPressure;
  return true;
}

/**
 * @brief Requests a measurement from the sensor, reads and calculates values.
 * 
 * @return 0 if successful, 1 if stale data, 2 if fault detected
 */
byte MS4515DO::poll_sensor() {
  byte aa, bb, cc, dd, status;

  _wire->requestFrom(_addr, 1); //Datasheet specifies 0 bytes but 1 works? Don't know why. 0 doesn't work though.
  _wire->read(); //Just make sure to read, in case the sensor erroneously returns something
  delay(2); //This is technically 0.5ms, let's just be safe though. This could be done via an interrupt triggered by the INT pin on the sensor
  _wire->requestFrom(_addr, 4);
  aa = _wire->read(); //read first byte
  bb = _wire->read(); //read second byte
  cc = _wire->read(); //read third byte
  dd = _wire->read(); //read fourth byte

  status = aa & B11000000; //Isolate the 2 MSB from the high transducer byte and
  status = status>>6; //shift so that they represent numbers 0, 1, 2 & 3.
  
  if (status == 2 || status == 3) return (status - 1); //If stale data or fault detected, return 1 for stale data, 2 for fault detected.

  aa = aa & B00111111; //Isolate the 6 LSB from the high transducer byte
  uint16_t transducer = aa; //Transducer is the value returned from the sensor
  transducer = transducer<<8; //Shift the byte into the MSB portion of the uint16 and
  transducer |= bb; //insert the low byte

  uint16_t temperature_raw = cc; //Temperature raw is the value returned from the sensor
  temperature_raw = temperature_raw<<8; //Basically the same as the transducer
  temperature_raw |= dd;
  temperature_raw = temperature_raw>>5;

  //Let's make some transfer functions!
  float press = (((transducer-819.15)*60.0/14744.7)-30.0)*248.84; //This is in Pa
  float temp = (temperature_raw*200.0/2047.0) - 50.0; //This is in °C

  pressure = press;
  temperature = temp;

  return 0;
}