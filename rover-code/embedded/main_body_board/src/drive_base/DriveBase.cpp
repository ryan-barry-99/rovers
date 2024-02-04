#include "../../include/DriveBase.h"

DriveBase::DriveBase()
    : wheels{
        Wheel(PWM_PIN_0, ENC_A_PIN_0, ENC_B_PIN_0, PIDConstants::KP0, PIDConstants::KI0, PIDConstants::KD0),
        Wheel(PWM_PIN_1, ENC_A_PIN_1, ENC_B_PIN_1, PIDConstants::KP1, PIDConstants::KI1, PIDConstants::KD1),
        Wheel(PWM_PIN_2, ENC_A_PIN_2, ENC_B_PIN_2, PIDConstants::KP2, PIDConstants::KI2, PIDConstants::KD2),
        Wheel(PWM_PIN_3, ENC_A_PIN_3, ENC_B_PIN_3, PIDConstants::KP3, PIDConstants::KI3, PIDConstants::KD3),
        Wheel(PWM_PIN_4, ENC_A_PIN_4, ENC_B_PIN_4, PIDConstants::KP4, PIDConstants::KI4, PIDConstants::KD4),
        Wheel(PWM_PIN_5, ENC_A_PIN_5, ENC_B_PIN_5, PIDConstants::KP5, PIDConstants::KI5, PIDConstants::KD5)
    }
{}

void DriveBase::updateVelocity() {
    for (int i = 0; i < 6; i++) {
        wheels[i].setSpeed(targetVelocity[i]);
    }
}
