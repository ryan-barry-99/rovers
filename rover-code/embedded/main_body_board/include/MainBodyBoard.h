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

class MainBodyBoard {
    public:
        /*
        * Constructor for the main body board class.
        * Initializes the drive base, temp subsystem, and LiDAR.
        */
        MainBodyBoard();
        ~MainBodyBoard();
    private:
        CAN can( CAN::CAN_ID::MAIN_BODY );
        DriveBase drive_base = new DriveBase(&can);
        TempSubsystem temp_subsystem = new TempSubsystem(&can);
};

#endif