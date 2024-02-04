#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "Motor.h"

template<typename T>
class StepperMotor<T> : public Motor<T>
{
    private:
    T currPos;
    T targetPos;
    int pin;
    Encoder encoder;

    public:

    // Constructor
    Motor();
    ~Motor(); // Virtual destructor to ensure proper cleanup


    setPower(float power);
    setPosition(T target);

    T getPosition(void);
    float getPower(void);

    stop(void);
}
#endif