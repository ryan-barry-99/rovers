#include "../include/CAN.h"
#include <FlexCAN_T4.h>

//#include <functional>

uint8_t m_CANMessage[8];

void CANSniff(const CAN_message_t &msg)
{
  msg.id;

  for(int i = 0; i < 8; i++)
  {
    m_CANMessage[i] = msg.buf[i];    
  } 
}

CAN::CAN(CAN::CAN_MB mailBox)
{
  //std::bind<void>(test, this);



  // m_CAN.setRX(CAN_pins::CAN_RX_PIN);
  // m_CAN.setTX(CAN_pins::CAN_TX_PIN);
  m_CAN.enableMBInterrupt((FLEXCAN_MAILBOX) mailBox);

  
  m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::MAIN_BODY,     TX, STD); // Set the mailbox to transmit
  m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::JETSON,        TX, STD); // Set the mailbox to transmit
  m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::SCIENCE_BOARD, TX, STD); // Set the mailbox to transmit
  m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::ARM_BOARD,     TX, STD); // Set the mailbox to transmit

  switch(mailBox){    
    case CAN::CAN_MB::MAIN_BODY:
      m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::MAIN_BODY, RX, STD); // Set the mailbox to receive
      break;
    
    case CAN_MB::JETSON:
      m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::JETSON, RX, STD); // Set the mailbox to receive
      break;

    case CAN::CAN_MB::SCIENCE_BOARD:
      m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::SCIENCE_BOARD, RX, STD); // Set the mailbox to receive
      break;
    
    case CAN::CAN_MB::ARM_BOARD:
      m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::ARM_BOARD, RX, STD); // Set the mailbox to receive
      break;
  }

// Start the CAN bus
  m_CAN.begin(); // <- This is needed

  // Set the baud rate to 115200
  m_CAN.setBaudRate(115200); 

  // Set the interrupt to call the canSniff function
  m_CAN.onReceive((FLEXCAN_MAILBOX)mailBox, CANSniff);
  
}

// Send a message to the CAN bus
void CAN::SendMessage( CAN_MB mailBox, uint32_t id, uint8_t message[8])
{
  // Create a message
  CAN_message_t msg;

  // Set the message ID to 0x123
  msg.id = id;
  
  // Set the message length to 8
  msg.len = 8;

  // Set the message buffer to the m_message buffer
  for(int i = 0; i < 8; i++)
  {
    msg.buf[i] = message[i];
  }

  // Send the message
  m_CAN.write( (FLEXCAN_MAILBOX)mailBox, msg);

}

auto CAN::GetMessage()
{

}