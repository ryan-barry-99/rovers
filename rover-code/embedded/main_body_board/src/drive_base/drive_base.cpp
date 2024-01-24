#include <Arduino.h>
#include "../include/drive_base.h"

DriveBase::DriveBase()
    : wheels{
        Wheel(PWM_PIN_0, ENC_A_PIN_0, ENC_B_PIN_0),
        Wheel(PWM_PIN_1, ENC_A_PIN_1, ENC_B_PIN_1),
        Wheel(PWM_PIN_2, ENC_A_PIN_2, ENC_B_PIN_2),
        Wheel(PWM_PIN_3, ENC_A_PIN_3, ENC_B_PIN_3),
        Wheel(PWM_PIN_4, ENC_A_PIN_4, ENC_B_PIN_4),
        Wheel(PWM_PIN_5, ENC_A_PIN_5, ENC_B_PIN_5)
    }
{
    // Set up drive base pins
    pinMode(PWM_PIN_0, OUTPUT);
    pinMode(PWM_PIN_1, OUTPUT);
    pinMode(PWM_PIN_2, OUTPUT);
    pinMode(PWM_PIN_3, OUTPUT);
    pinMode(PWM_PIN_4, OUTPUT);
    pinMode(PWM_PIN_5, OUTPUT);

    pinMode(ENC_A_PIN_0, INPUT);
    pinMode(ENC_A_PIN_1, INPUT);
    pinMode(ENC_A_PIN_2, INPUT);
    pinMode(ENC_A_PIN_3, INPUT);
    pinMode(ENC_A_PIN_4, INPUT);
    pinMode(ENC_A_PIN_5, INPUT);

    pinMode(ENC_B_PIN_0, INPUT);
    pinMode(ENC_B_PIN_1, INPUT);
    pinMode(ENC_B_PIN_2, INPUT);
    pinMode(ENC_B_PIN_3, INPUT);
    pinMode(ENC_B_PIN_4, INPUT);
    pinMode(ENC_B_PIN_5, INPUT);
}

void DriveBase::updateVelocity() {
    for (int i = 0; i < 6; i++) {
        wheels[i].setSpeed(targetVelocity[i]);
    }
}