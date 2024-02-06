#ifndef SCIENCE_BOARD_H
#define SCIENCE_BOARD_H
#include <Arduino.h>
#include "SoilCollector.h"
#include "SoilTransferer.h"
#include "Flurometer.h"
#include "CAN.h"
#include "EnumList.h"

// Attributes
// - soilCollector: SoilCollector
// - soilTransferer: SoilTransferer
// - fluorometer: Fluorometer
// - can: CAN

class ScienceBoard
{
    private:
        CAN can = CAN( CAN_MB::SCIENCE_BOARD );
        SoilCollector soilCollector = SoilCollector( &can);
        SoilTransferer soilTransferer = SoilTransferer( &can);;
        Flurometer fluromete = Flurometer( &can);
        
public:
    ScienceBoard();
    ~ScienceBoard();
};
#endif