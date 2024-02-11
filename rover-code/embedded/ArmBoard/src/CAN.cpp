#include "../include/CAN.h"


CAN::CAN(CAN_MB mailBox)
{
  m_CAN.enableMBInterrupt((FLEXCAN_MAILBOX) mailBox);

  
  m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::MAIN_BODY,     TX, STD); // Set the mailbox to transmit
  m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::JETSON,        TX, STD); // Set the mailbox to transmit
  m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::SCIENCE_BOARD, TX, STD); // Set the mailbox to transmit
  m_CAN.setMB( (FLEXCAN_MAILBOX)CAN::ARM_BOARD,     TX, STD); // Set the mailbox to transmit

  m_CAN.setMB( (FLEXCAN_MAILBOX)mailBox, RX, STD); // Set the mailbox to receive

  // Start the CAN bus
  m_CAN.begin(); // <- This is needed

  // Set the baud rate to 500000
  m_CAN.setBaudRate(500000); 

  // Set the interrupt to call the canSniff function
  m_CAN.onReceive((FLEXCAN_MAILBOX)mailBox, &CAN::CANSniff);
  
}

void CAN::CANSniff(const CAN_message_t &msg)
{
  m_objectDict[msg.id] = msg;
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

  // Add the message to the object dictionary
  m_objectDict[msg.id] = msg;

  // Send the message
  m_CAN.write( (FLEXCAN_MAILBOX)mailBox, msg);

}

// Retrieve a message from the object dictionary
auto CAN::GetMessage(uint32_t id)
{
  return m_objectDict[id];
}