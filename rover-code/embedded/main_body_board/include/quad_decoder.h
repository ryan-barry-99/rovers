#ifndef QUAD_DECODER_H
#define QUAD_DECODER_H

#include "pinout.h"
#include <Encoder.h>
#include <IntervalTimer.h>

class QuadratureDecoder {
public:
  QuadratureDecoder(enc_A_pins enc_A_pin, enc_B_pins enc_B_pin);
  void begin();
  long getCount();

private:
  Encoder m_encoder;
  volatile long m_count;
  int enc_A_pin;
  int enc_B_pin;
  static QuadratureDecoder* instance;
  static void updateCount();
};

#endif