/*
 * Generic PID library for use in Arduino
 * 
 * Written by Mani Magnusson
 */

#ifndef _PID_H
#define _PID_H

#include <Arduino.h>

class PID {
    public:
        PID(float Kp, float Kd, float Ki);
    private:
        float KpGain;
        float KdGain;
        float KiGain;
};

#endif