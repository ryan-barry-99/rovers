#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Servo.h>
#include "motor.h"
#include "Pinout.h"

class Motor {
    public:
        Motor();
        Motor(pwm_pins pwm_pin);
        void setSpeed(float duty_cycle_us);
    private:
        pwm_pins pwm_pin;
        Servo motor;
};
#endif