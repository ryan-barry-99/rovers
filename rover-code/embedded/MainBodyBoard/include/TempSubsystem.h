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

class TempSubsystem {
    public:
        /*
        * Constructor for the temp subsystem class.
        * Initializes the thermistors.
        */
        TempSubsystem(CAN *can);

        /*
        * @return The temperature measured by the thermistor
        */
        float* getTemperature();
        /*
         * Set the power of the fans
        */
        void setFansPower(int power);
        /*
         * Update the temperature and fan power
        */
        void update(void);
    private:
        /*
        * The thermistors
        */
        Thermistor thermistors[NUM_THERMISTORS];
        /*
        * The fans
        */
        Fan fans[NUM_FANS];
        float temperature[NUM_THERMISTORS];
};

#endif