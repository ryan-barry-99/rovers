#include  "../../include/motor.h"


Motor::Motor(pwm_pins pwm_pin) {
    this->pwm_pin = pwm_pin;
    pinMode(this->pwm_pin, OUTPUT);
    Servo motor;
    motor.attach(this->pwm_pin);
}


Motor::setSpeed(float duty_cycle_us) {
    motor.writeMicroseconds(duty_cycle_us);
}