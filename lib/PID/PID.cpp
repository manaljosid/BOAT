/*
 * Generic PID library for use in Arduino
 * 
 * Written by Mani Magnusson
 */

#include <PID.h>
#include <Arduino.h>

PID::PID(float Kp, float Kd, float Ki) {
    KpGain = Kp;
    KdGain = Kd;
    KiGain = Ki;
}