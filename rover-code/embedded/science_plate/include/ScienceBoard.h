#ifndef SCIENCE_BOARD_H
#define SCIENCE_BOARD_H
#include <Arduino.h>
#include "SoilCollector.h"
#include "SoilTransferer.h"
#include "Fluorometer.h"
#include "CAN.h"

// Attributes
// - soilCollector: SoilCollector
// - soilTransferer: SoilTransferer
// - fluorometer: Fluorometer
// - can: CAN

class ScienceBoard
{
    private:
        CAN can( CAN::CAN_ID::SCIENCE_BOARD );
        SoilCollector soilCollector = new SoilCollector(&can);
        SoilTransferer soilTransferer = new SoilTransferer(&can);
        Fluorometer fluorometer = new Fluorometer(&can);
        
public:
    ScienceBoard();
};
#endif