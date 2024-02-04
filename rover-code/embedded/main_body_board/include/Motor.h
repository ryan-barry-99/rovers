#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Servo.h>
#include "Motor.h"
#include "Pinout.h"
#include "PIDController.h"

class Motor {
    public:
        Motor();
        Motor(pwm_pins pwm_pin, double kp, double ki, double kd);
        void setSpeed(float duty_cycle_us);
    private:
        pwm_pins pwm_pin;
        Servo motor;
        PIDController pid;
};
#endif
