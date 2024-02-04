#include "../../include/SoilCollector.h"

SoilCollector::SoilCollector(CAN *can) 
{
    this->can = can;
  // Set the default values for the soil collector
    TurnDirection turnDirection = TurnDirection::CLOCKWISE;
    
    bool extended = false;
    bool turning = false;
}

bool isExtended(){return extended;}

bool isTurning(){return turning;}

TurnDirection getTurnDirection(){return turnDirection;}

void setTurnDirection(TurnDirection direction){turnDirection = direction;}