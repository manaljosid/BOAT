/*
 *  Library for use with the TE Measurements 4515DO-DS3BK030DPL differential pressure sensor.
 *  The sensor communicates over I2C and uses a 3.3V supply.
 *  It is intended to be used as an airspeed sensor onboard model planes.
 *  
 *  Author: Mani Magnusson
 */

#ifndef _MS4515DO_H
#define _MS4515DO_H

#include <Arduino.h>
#include <Wire.h>

#define MS4515DO_ADDRESS (0x46)

//Forward declaration
extern TwoWire Wire;

class MS4515DO {
    public:
        MS4515DO(TwoWire *theWire = &Wire);

        bool begin(uint8_t addr);
        byte poll_sensor();
        float read_pressure();
        float read_temperature();
        bool zero();

        TwoWire *_wire;
    private:
        float pressure;
        float temperature;
        float offsetPressure;

        int _addr;
};

#endif