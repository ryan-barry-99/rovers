#include "../../include/Thermistor.h"

Thermistor::Thermistor(thermistor_pins thermistor_pin) {
    this->thermistor_pin = thermistor_pin;
    pinMode(thermistor_pin, INPUT);
}

float Thermistor::getTemperature() {
    float voltage = analogRead(thermistor_pin) * 3.3 / 1023.0;
    float resistance = 10000.0 * voltage / (3.3 - voltage);
    // Steinhart-Hart equation
    // https://en.wikipedia.org/wiki/Thermistor#B_or_%CE%B2_parameter_equation
    // See if we want to use this or approximation scale from datasheet
    this->temperature = 1.0 / (log(resistance / 10000.0) / 3950.0 + 1.0 / 298.15) - 273.15;
    return this->temperature;
}
