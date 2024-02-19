#include "../include/MainBodyBoard.h"

MainBodyBoard::MainBodyBoard()
{
    this->can = CAN( CAN::CAN_MB::SCIENCE_BOARD );
    this->drive_base = DriveBase(&can);
    this->temp_subsystem = TempSubsystem(&can);
}

MainBodyBoard::~MainBodyBoard()
{
    
}

void MainBodyBoard::updateSubsystems(void)
{
    drive_base.updateVelocity();
    temp_subsystem.updateFans();
}

