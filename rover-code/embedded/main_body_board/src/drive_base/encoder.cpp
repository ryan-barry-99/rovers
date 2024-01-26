#include "../include/encoder.h"

Encoder::Encoder(enc_A_pins enc_A_pin, enc_B_pins enc_B_pin)
{
    this->enc_A_pin = enc_A_pin;
    this->enc_B_pin = enc_B_pin;
    this->m_encoder = new Encoders(this->enc_A_pin, this->enc_B_pin);
}

int Encoder::getSpeed()
{
    return m_encoder.getEncoderCount();
}

void Encoder::setCount(long count)
{
    Encoders::setEncoderCount(count);
}