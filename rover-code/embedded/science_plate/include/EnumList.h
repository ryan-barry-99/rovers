#ifndef ENUM_LIST_H
#define ENUM_LIST_H

enum FluroPos{
    POS0 = 0,
    POS1 = 1,
    POS2 = 2,
    POS3 = 3,
    POS4 = 4,
    POS5 = 5,
};

enum PosSoilTransfer{
    POS0 = 0,
    POS1 = 1,
    POS2 = 2,
    POS3 = 3,
    POS4 = 4,
    POS5 = 5,
};

enum StepperPins{
    ID0 = 0,
    ID1 = 1,
    ID2 = 2,
    ID3 = 3,
    ID4 = 4,
    ID5 = 5,
};

enum VibratorPins{
    ID0 = 0,
    ID1 = 1,
    ID2 = 2,
    ID3 = 3,
    ID4 = 4,
    ID5 = 5,
};

enum TurnDirection{
    CLOCKWISE = 0,
    COUNTERCLOCKWISE = 1
};

enum CAN_MB{
    JETSON = FLEXCAN_MAILBOX::MB0, 
    MAIN_BODY = FLEXCAN_MAILBOX::MB1, 
    SCIENCE_BOARD = FLEXCAN_MAILBOX::MB2,
    ARM_BOARD = FLEXCAN_MAILBOX::MB3
};

#endif