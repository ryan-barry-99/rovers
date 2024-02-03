#include "../include/ScienceBoard.h"

ScienceBoard::ScienceBoard()
{
    this->can = CAN( CAN::CAN_ID::SCIENCE_BOARD );
    this->soilCollector = SoilCollector(&can);
    this->soilTransferer = SoilTransferer(&can);
    this->fluorometer = Fluorometer(&can);
}

ScienceBoard::~ScienceBoard()
{
    delete this->soilCollector;
    delete this->soilTransferer;
    delete this->fluorometer;
}