/*
File: wheel.h
Author: Ryan Barry
Date Created: 1/23/2024

This file defines the wheel class for the rover.

This class is responsible for controlling each individual wheel of the rover 
based on its target speed.
*/

#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>
#include <Servo.h>
#include "pinout.h"
#include "wheel.h"
#include "motor.h"
#include "quad_decoder.h"

class Wheel {
    public:
        /*
        * Constructor for the wheel class
        * @param pwm_pin The PWM pin the wheel is connected to
        * @param enc_A_pin The A pin of the wheel's quadrature encoder
        * @param enc_B_pin The B pin of the wheel's quadrature encoder
        */
        Wheel(pwm_pins pwm_pin, enc_A_pins enc_A_pin, enc_B_pins enc_B_pin);

        /*
        * Adjust's the PWM signal to the wheel to match the target speed
        * @param targetSpeed The target speed of the wheel
        */
        void setSpeed(float targetSpeed);

        /*
        * Gets the current speed of the wheel
        * @return The current speed of the wheel
        */
        int getSpeed();
    private:
        float currentSpeed;
        float targetSpeed;
        Motor motor;
        QuadratureDecoder encoder;
        float pwm_duty_cycle;
};

#endif