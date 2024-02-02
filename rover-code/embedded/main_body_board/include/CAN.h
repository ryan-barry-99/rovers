#ifndef CAN_H
#define CAN_H
#include <FlexCAN_T4.h>
#include "pinout.h"
#include "object_dict.h"

class CAN
{
    public:    
    
    enum CAN_MB{
        JETSON = FLEXCAN_MAILBOX::MB0, 
        MAIN_BODY = FLEXCAN_MAILBOX::MB1, 
        SCIENCE_BOARD = FLEXCAN_MAILBOX::MB2,
        ARM_BOARD = FLEXCAN_MAILBOX::MB3
        };


    CAN(CAN::CAN_MB mailBox);

    void SendMessage( CAN_MB mailBox, uint32_t id, uint8_t message[8]);

    // void canSniff(const CAN_message_t &msg);
    auto GetMessage();

    // static void staticCanSniff(const CAN_message_t &msg);

    ObjectDictionary m_objectDict;
    ObjectDictionary::Type
    private:
    
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> m_CAN;
};
#endif