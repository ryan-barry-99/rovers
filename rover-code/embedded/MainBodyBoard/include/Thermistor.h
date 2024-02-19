/*
File: thermistor.h
Author: Ryan Barry
Date Created: 1/23/2024

This file defines the thermistor class for the rover.
This class is responsible for reading the temperature of a thermistor.
*/

#ifndef THERMISTOR_H
#define THERMISTOR_H

#define MAX11607_ADDRESS 0b0110100
#define MAX11607_VREF 2.048

// MAX11607 Setup
// bit:7 = setup bit = 1
// bit:6 = sel2
// bit:5 = sel1
// bit:4 = sel0
// bit:3 = clk - 1 = external clock, 0 = internal clock
// bit:2 = unipolar - 0 = unipolar, 1 = bipolar
// bit:1 = reset configuration 
// bit:0 = useless

// MAC11607 output
// 1 LSB = 2.048 / 1024 = 2mV
// 10 bits of data
// first 6 bits in the first byte are useless

#include <Wire.h>

#include <Arduino.h>
#include "Pinout.h"

class Thermistor {
    public:
        /*
        * Constructor for the thermistor class.
        * Initializes the thermistor.
        */
        Thermistor(THERMISTOR_PINS thermistorPin);

        /*
        * @return The temperature measured by the thermistor
        */
        float getTemperature();
    private:
        /*
        * The pin that the thermistor is connected to
        */
        THERMISTOR_PINS m_thermistorPin;
        float m_temperature;
};

#endif
