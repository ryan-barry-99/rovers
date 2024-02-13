#include "../../include/TempSubsystem.h"
#include "../../include/Pinout.h"

TempSubsystem::TempSubsystem(CAN *can) :
    thermistors
    {
        Thermistor(thermistor_pins::THERMISTOR_PIN_0), 
        Thermistor(thermistor_pins::THERMISTOR_PIN_1),
        Thermistor(thermistor_pins::THERMISTOR_PIN_2),
        Thermistor(thermistor_pins::THERMISTOR_PIN_3)
    },
    fans
    {
        Fan(fan_pins::FAN_PIN_0),
        Fan(fan_pins::FAN_PIN_1),
        Fan(fan_pins::FAN_PIN_2),
        Fan(fan_pins::FAN_PIN_3)
    }
{}


float* TempSubsystem::getTemperature() {
    this->temperature[0] = thermistors[0].getTemperature();
    this->temperature[1] = thermistors[1].getTemperature();
    this->temperature[2] = thermistors[2].getTemperature();
    this->temperature[3] = thermistors[3].getTemperature();
    return this->temperature;
}

void TempSubsystem::setFansPower(int power)
{
    if(power > MAX_FAN_SPEED)
    {
        power = MAX_FAN_SPEED;
    }
    else if(power < MIN_FAN_SPEED)
    {
        power = MIN_FAN_SPEED;
    }

    for (int i = 0; i < 4; i++)
    {
        fans[i].setPower(power);
    }
}

void TempSubsystem::update()
{
    getTemperature();
}