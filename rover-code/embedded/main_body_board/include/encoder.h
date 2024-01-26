#pragma once
#include "pinout.h"
#include <QuadratureEncoder.h>

class Encoder
{
    public:
    Encoder(enc_A_pins enc_A_pin, enc_B_pins enc_B_pin);

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