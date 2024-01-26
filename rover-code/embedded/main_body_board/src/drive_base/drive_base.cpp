#include "../../include/drive_base.h"

DriveBase::DriveBase()
    : wheels{
        Wheel(PWM_PIN_0, ENC_A_PIN_0, ENC_B_PIN_0),
        Wheel(PWM_PIN_1, ENC_A_PIN_1, ENC_B_PIN_1),
        Wheel(PWM_PIN_2, ENC_A_PIN_2, ENC_B_PIN_2),
        Wheel(PWM_PIN_3, ENC_A_PIN_3, ENC_B_PIN_3),
        Wheel(PWM_PIN_4, ENC_A_PIN_4, ENC_B_PIN_4),
        Wheel(PWM_PIN_5, ENC_A_PIN_5, ENC_B_PIN_5)
    }
{}

void DriveBase::updateVelocity() {
    for (int i = 0; i < 6; i++) {
        wheels[i].setSpeed(targetVelocity[i]);
    }
}
