#ifndef FLUROMETER_H
#define FLUROMETER_H

#include "CAN.h"
#include "MicroPump.h"
#include "FluroPos.h"
#include "StepperMotor.h"
#include "Enums.h"

class Flurometer
{
private:
    CAN* can;
    MicroPump pump;
    FluroPos position;
    NEMA17Stepper stepper = new Motor(); 

    float photoDiode;
    int diodeLEDPin;     
    bool pumping;
    
    
public:
    Flurometer(CAN* can);
    ~Flurometer();
};




#endif // FLUROMETER_H