#ifndef FAN_H
#define FAN_H

#include "Pinout.h"
#include <Arduino.h>

class Fan
{
    public:
    Fan(fan_pins pin);
    void setPower(int power);
    //pwm pin to control fan speed
    // four fans
    private:
    fan_pins pin;
};

#endif