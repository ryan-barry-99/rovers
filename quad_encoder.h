#ifndef QUAD_ENCODER_H
#define QUAD_ENCODER_H
#include "pinout.h"
#include <Arduino.h>
#include <QuadratureEncoder.h>

class QuadEncoder
{
    public:
        QuadEncoder(enc_A_pins enc_A_pin, enc_B_pins enc_B_pin);

        /*
        * gets the velocity of the encoder (RPM)
        */
        int getSpeed();
        void setCount(long count);

    private:
        enc_A_pins enc_A_pin;
        enc_B_pins enc_B_pin;
        Encoders m_encoder;

};

#endif