#ifndef SOIL_COLLECTOR_H
#define SOIL_COLLECTOR_H

#include "StepperMotor.h"
#include "BrushlessMotor.h"
#include "CAN.h"
#include "Enums.h"

// Attributes
// - stepper: StepperMotor
// - brushless: BrushlessMotor
// - extended: bool
// - turning: bool
// - turnDirection: TurnDirection
// - can: *CAN

class SoilCollector
{
private:
    StepperMotor stepper = new StepperMotor();
    BrushlessMotor brushless = new BrushlessMotor();

    TurnDirection turnDirection;

    bool extended;
    bool turning;
    
    CAN *can;
public:
    SoilCollector(CAN *can);

    isExtended(): bool
    bool isTurning()
    getTurnDirection(): int
    setTurnDirection(direction: TurnDirection)
};

#endif