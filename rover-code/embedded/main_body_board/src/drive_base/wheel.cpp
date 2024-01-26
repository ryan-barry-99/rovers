#include <Arduino.h>
#include <Servo.h>
#include "../include/wheel.h"
#include "../include/pinout.h"

Wheel::Wheel(pwm_pins pwm_pin, enc_A_pins enc_A_pin, enc_B_pins enc_B_pin) {
    this->pwm_pin = pwm_pin;
    this->enc_A_pin = enc_A_pin;
    this->enc_B_pin = enc_B_pin;
    this->targetSpeed = 0;
    this->currentSpeed = 0;

    // Configure pins as outputs or inputs
    pinMode(this->pwm_pin, OUTPUT);
    pinMode(this->enc_A_pin, INPUT);
    pinMode(this->enc_B_pin, INPUT);
}

void Wheel::setSpeed(float targetSpeed) {
    this->targetSpeed = targetSpeed;
    /*
    Do something here with controls for PWM
    https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces

    Full Reverse 1000 us pulse width
    Proportional Reverse 1000 to 1475 us pulse width
    Neutral 1475 to 1525 us pulse width
    Proportional Forward 1525 to 2000 us pulse width
    Full Forward 2000 us pulse width

    Maximum pulse range 500 to 2500 us
    Valid frequency 50 to 200 Hz
    */
   analogWrite(this->pwm_pin, this->pwm_duty_cycle);
}
