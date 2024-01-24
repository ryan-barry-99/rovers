#include "../lib/temp_subsystem.h"
#include "../lib/pinout.h"

TempSubsystem::TempSubsystem() :
    thermistors
    {
        Thermistor(thermistor_pins::THERMISTOR_PIN_0), 
        Thermistor(thermistor_pins::THERMISTOR_PIN_1)
    }
{}


float* TempSubsystem::getTemperature() {
    this->temperature[0] = thermistors[0].getTemperature();
    this->temperature[1] = thermistors[1].getTemperature();
    return this->temperature;
}