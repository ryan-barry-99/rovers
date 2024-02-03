#ifndef ENUM_LIST_H
#define ENUM_LIST_H

enum FLURO_POS{
    POS0 = 0,
    POS1 = 1,
    POS2 = 2,
    POS3 = 3,
    POS4 = 4,
    POS5 = 5,
};

enum POS_SOIL_TRANSFER{
    POS0 = 0,
    POS1 = 1,
    POS2 = 2,
    POS3 = 3,
    POS4 = 4,
    POS5 = 5,
};

enum TURN_DIRECTION{
    CLOCKWISE = 0,
    COUNTERCLOCKWISE = 1
};

enum CAN_MB{
    JETSON = FLEXCAN_MAILBOX::MB0, 
    MAIN_BODY = FLEXCAN_MAILBOX::MB1, 
    SCIENCE_BOARD = FLEXCAN_MAILBOX::MB2,
    ARM_BOARD = FLEXCAN_MAILBOX::MB3
};

enum MICROPUMP_MODE{
    DISCRETE = 0,
    VARIABLE = 1
};

#endif