#include "../../include/DriveBase.h"

/*
* Constructor for the drive base class.
* Initializes the wheels of the rover.
*/
DriveBase::DriveBase(CAN *can)
    : wheels{
        Wheel(PWM_PIN_0, ENC_A_PIN_0, ENC_B_PIN_0, PIDConstants::KP0, PIDConstants::KI0, PIDConstants::KD0),
        Wheel(PWM_PIN_1, ENC_A_PIN_1, ENC_B_PIN_1, PIDConstants::KP1, PIDConstants::KI1, PIDConstants::KD1),
        Wheel(PWM_PIN_2, ENC_A_PIN_2, ENC_B_PIN_2, PIDConstants::KP2, PIDConstants::KI2, PIDConstants::KD2),
        Wheel(PWM_PIN_3, ENC_A_PIN_3, ENC_B_PIN_3, PIDConstants::KP3, PIDConstants::KI3, PIDConstants::KD3),
        Wheel(PWM_PIN_4, ENC_A_PIN_4, ENC_B_PIN_4, PIDConstants::KP4, PIDConstants::KI4, PIDConstants::KD4),
        Wheel(PWM_PIN_5, ENC_A_PIN_5, ENC_B_PIN_5, PIDConstants::KP5, PIDConstants::KI5, PIDConstants::KD5)
    }
{
    m_can = can;
}

// Retrieves the target velocity from the CAN bus
void DriveBase::getTargetVelocity()
{
    CANFD_message_t msg = m_can->GetMessage(CAN::Message_ID::TARGET_VELOCITY);
    for (int i = 0; i < 6; i++) {
        targetVelocity[i] = (float)msg.buf[i]; // This line will change based on message packing
    }
}

// Updates the velocity of the wheels to match the target velocity
void DriveBase::updateVelocity()
{
    getTargetVelocity();
    for (int i = 0; i < 6; i++) {
        wheels[i].setSpeed(targetVelocity[i]);
    }
}
