#include "../../include/Wheel.h"

Wheel::Wheel(pwm_pins pwm_pin, enc_A_pins enc_A_pin, enc_B_pins enc_B_pin) : motor(pwm_pin), encoder(enc_A_pin, enc_B_pin) {

    this->targetSpeed = 0;
    this->currentSpeed = 0;
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
   motor.setSpeed(targetSpeed);
}