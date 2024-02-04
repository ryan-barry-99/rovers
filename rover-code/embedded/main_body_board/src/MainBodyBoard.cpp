#include "../include/MainBodyBoard.h"

MainBodyBoard::MainBodyBoard()
{
    this->can = CAN( CAN::CAN_ID::SCIENCE_BOARD );
    this->drive_base = DriveBase(&can);
    this->temp_subsystem = TempSubsystem(&can);
}

MainBodyBoard::~MainBodyBoard()
{
    delete this->drive_base;
    delete this->temp_subsystem;
}