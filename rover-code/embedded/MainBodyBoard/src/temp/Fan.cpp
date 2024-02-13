#include "../include/Fan.h"

Fan::Fan(fan_pins pin)
{
    this->pin = pin;
    pinMode(pin, OUTPUT);
    //pwm pin to control fan speed
}

void Fan::setPower(int power)
{
    analogWrite(pin, power);
    // four fans
}