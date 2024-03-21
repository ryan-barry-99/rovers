/*
File: main_body_board.h
Author: Ryan Barry
Date Created: 1/23/2024

This file defines the main body board class for the rover.

The main body board is responsible for:
- controlling the drive wheels with encoder feedback
- reading the temperature of the thermistors
*/

#ifndef MAIN_BODY_BOARD_H
#define MAIN_BODY_BOARD_H

#include "CAN.h"
#include "DriveBase.h"
#include "TempSubsystem.h"
#include "DEBUG.h"
#include "Pinout.h"

class MainBodyBoard {
    public:
        /*
        * Constructor for the main body board class.
        * Initializes the drive base, temp subsystem, and LiDAR.
        */
        MainBodyBoard();
        ~MainBodyBoard();

        void updateSubsystems(void);
    private:
        #ifndef DEBUG_STATUS_LIGHT
        bool statusLightOn = false;
        int statusLightWait = 0;
        #endif

        #ifndef DEBUG_CAN
        CAN can = CAN( CAN::CAN_MB::MAIN_BODY );
        #endif

        #ifndef DEBUG_DRIVEBASE || DEBUG_CAN
        DriveBase drive_base = DriveBase(&can);
        #endif

        #ifndef DEBUG_TEMP || DEBUG_CAN
        TempSubsystem temp_subsystem = TempSubsystem(&can);
        #endif
};

#endif