#include "../../include/TempSubsystem.h"


/*
* Constructor for the temp subsystem class.
* Initializes the thermistors.
*/
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
{
    m_can = can;
}


// @return An array of temperatures measured by each thermistor
float* TempSubsystem::getTemperature() 
{
    for(int i=0; i<NUM_THERMISTORS; i++)
    {
        this->temperature[i] = this->thermistors[i].getTemperature();
    }
    return this->temperature;
}

// Set the power of the fans
void TempSubsystem::setFansPower(int power)
{
    power = min(max(power, MIN_FAN_SPEED), MAX_FAN_SPEED); // clamp the power to the range (MIN_FAN_SPEED, MAX_FAN_SPEED)

    for (int i = 0; i < NUM_FANS; i++)
    {
        this->fans[i].setPower(power);
    }
}

void TempSubsystem::updateFans()
{
    getTemperature();
    float avgTemp = 0;
    for (int i = 0; i < NUM_THERMISTORS; i++)
    {
        avgTemp += temperature[i];
    }
    avgTemp /= NUM_THERMISTORS;
    if(avgTemp > MAX_TEMP)
    {
        setFansPower(MAX_FAN_SPEED);
    }
    else if(avgTemp < MIN_TEMP)
    {
        setFansPower(MIN_FAN_SPEED);
    }
    else
    {

        setFansPower(0); // change later
    }
}