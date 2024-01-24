#include <Arduino.h>
#include "../lib/wheel.h"
#include "../lib/pinout.h"

Wheel::Wheel(pwm_pins pwm_pin, enc_A_pins enc_A_pin, enc_B_pins enc_B_pin) {
    this->pwm_pin = pwm_pin;
    this->enc_A_pin = enc_A_pin;
    this->enc_B_pin = enc_B_pin;
    this->targetSpeed = 0;
    this->currentSpeed = 0;
}

void Wheel::setSpeed(float targetSpeed) {
    this->targetSpeed = targetSpeed;
    /*
    Do something here with controls for PWM
    */
   analogWrite(this->pwm_pin, this->pwm_duty_cycle);
}