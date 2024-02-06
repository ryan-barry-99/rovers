#ifndef MICROPUMP_H
#define MICROPUMP_H

#include "EnumList.h"

/*
Attributes
- pumping: bool
- mode: MICROPUMP_MODE
- pulsesPerSecond: int
- normalizedSpeed: float
*/

class MicroPump
{
private:
    bool pumping;
    MICROPUMP_MODE mode;
    int pulsesPerSecond;
    float normalizedSpeed;
    CAN *can;

public:
    MicroPump(CAN *can);

    float ppsToNormal(int pulses);
    int normalToPps(float normalVal);

    int setNormal(float normalVal);
    int setPps(int pulses);

    int getPps();
    float getNormal();
};

#endif