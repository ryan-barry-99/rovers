#include "../../include/quad_encoder.h"

QuadEncoder::QuadEncoder(enc_A_pins enc_A_pin, enc_B_pins enc_B_pin) : m_encoder(enc_A_pin, enc_B_pin)
{
    this->enc_A_pin = enc_A_pin;
    this->enc_B_pin = enc_B_pin;
}

int QuadEncoder::getSpeed()
{
    return m_encoder.getEncoderCount();
}

void QuadEncoder::setCount(long count)
{
    m_encoder.setEncoderCount(count);
}