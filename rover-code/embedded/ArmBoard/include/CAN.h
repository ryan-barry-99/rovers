#ifndef CAN_H
#define CAN_H
#include <FlexCAN_T4.h>
#include <unordered_map>
#include "Pinout.h"

using ObjectDictionary = std::unordered_map<uint32_t, CAN_message_t>;

class CAN
{
    public:    
        enum CAN_MB{
            JETSON = FLEXCAN_MAILBOX::MB0, 
            MAIN_BODY = FLEXCAN_MAILBOX::MB1, 
            SCIENCE_BOARD = FLEXCAN_MAILBOX::MB2,
            ARM_BOARD = FLEXCAN_MAILBOX::MB3
            };


        CAN(CAN_MB mailBox);

        void SendMessage( CAN_MB mailBox, uint32_t id, uint8_t message[8]);
        auto GetMessage(uint32_t id);

    private:
        FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> m_CAN;
        static ObjectDictionary m_objectDict;
        static void CANSniff(const CAN_message_t &msg);
};
#endif