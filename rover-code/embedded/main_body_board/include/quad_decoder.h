#ifndef QUAD_DNCODER_H
#define QUAD_DNCODER_H
#include "pinout.h"
#include <Arduino.h>
#include <Encoder.h>
#include <IntervalTimer.h>

class QuadratureDecoder
{
    public:
        QuadratureDecoder(enc_A_pins enc_A_pin, enc_B_pins enc_B_pin);

        /*
        * gets the velocity of the encoder (RPM)
        */
        void begin();
        long getCount();

    private:
        Encoder m_encoder;

};

#endif