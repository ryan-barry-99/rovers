#include "../include/Flurometer.h"
#include "../include/Enums.h"

Flurometer::Flurometer(CAN* can)
{
    this->can = can;
    this->pump = MicroPump();
    this->position = FluroPos.POS0;
    this->stepper = NEMA17Stepper();
    this->photoDiode = 0;
    this->diodeLEDPin = 0;
    this->pumping = false;
}

Flurometer::~Flurometer()
{
    delete this->can;
    delete this->pump;
    delete this->stepper;
}