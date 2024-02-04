/*
File: temp_subsystem.h
Author: Ryan Barry
Date Created: 1/23/2024

This file defines the temp subsystem class for the rover.

This class is responsible for reading the temperature of the thermistors.
*/

#ifndef TEMP_SUBSYSTEM_H
#define TEMP_SUBSYSTEM_H

#include "Thermistor.h"
#include "Pinout.h"

class TempSubsystem {
    public:
        /*
        * Constructor for the temp subsystem class.
        * Initializes the thermistors.
        */
        TempSubsystem(CAN::CAN *can);

        /*
        * @return The temperature measured by the thermistor
        */
        float* getTemperature();
    private:
        /*
        * The thermistors
        */
        Thermistor thermistors[2];
        float temperature[2];
};

#endif