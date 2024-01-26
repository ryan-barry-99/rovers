/*
File: drive_base.h
Author: Ryan Barry
Date Created: 1/23/2024

This file defines the drive base class for the rover.

This class is responsible for controlling the rover's wheels based on 
the target velocity transmitted over CAN.
*/

#ifndef DRIVE_BASE_H
#define DRIVE_BASE_H

#include <Arduino.h>
#include "wheel.h"
#include "pinout.h"

class DriveBase {
    public:
        /*
        * Constructor for the drive base class.
        * Initializes the wheels of the rover.
        */
        DriveBase();

        /*
        * Updates the velocity of the wheels to match the target velocity
        */
        void updateVelocity();
    private:
        /*
        * An array of the rover's wheels
        */
        Wheel wheels[6];

        /*
        * An array of the target velocities corresponding to each wheel
        */
        float targetVelocity[6];

};
#endif