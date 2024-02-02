#ifndef PINOUT_H
#define PINOUT_H

enum pwm_pins {
    PWM_PIN_0 = 2,
    PWM_PIN_1 = 3,
    PWM_PIN_2 = 4,
    PWM_PIN_3 = 5,
    PWM_PIN_4 = 6,
    PWM_PIN_5 = 7
};

enum enc_A_pins {
    ENC_A_PIN_0 = 8,
    ENC_A_PIN_1 = 9,
    ENC_A_PIN_2 = 10,
    ENC_A_PIN_3 = 11,
    ENC_A_PIN_4 = 12,
    ENC_A_PIN_5 = 13
};

enum enc_B_pins {
    ENC_B_PIN_0 = 14,
    ENC_B_PIN_1 = 15,
    ENC_B_PIN_2 = 16,
    ENC_B_PIN_3 = 17,
    ENC_B_PIN_4 = 18,
    ENC_B_PIN_5 = 19
};

enum thermistor_pins {
    THERMISTOR_PIN_0 = 20,
    THERMISTOR_PIN_1 = 21
};

enum CAN_pins {
    CAN_RX_PIN = 0,
    CAN_TX_PIN = 1
};

#endif