/*
File: temp_subsystem.h
Author: Ryan Barry
Date Created: 1/23/2024

This file defines the temp subsystem class for the rover.

This class is responsible for reading the temperature of the thermistors.
*/

#ifndef TEMP_SUBSYSTEM_H
#define TEMP_SUBSYSTEM_H

#define NUM_THERMISTORS 4
#define NUM_FANS 4

#define MAX_TEMP 110
#define MIN_TEMP 60

#define MAX_FAN_SPEED 255
#define MIN_FAN_SPEED 511

#include "Fan.h"
#include "Thermistor.h"
#include "Pinout.h"
#include "CAN.h"

using namespace std;

class TempSubsystem {
    public:
        TempSubsystem(CAN *can);

        float* getTemperature();
        void setFansPower(int power);
        void update(void);
    private:
        // Array of Thermistor objects
        Thermistor thermistors[NUM_THERMISTORS];

        // Array of Fan objects
        Fan fans[NUM_FANS];

        // Array of temperature readings
        float temperature[NUM_THERMISTORS];

        // Pointer to the CAN object
        CAN *m_can;
};

#endif