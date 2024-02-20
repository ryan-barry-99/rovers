"""
File: ArmMotor.py

Description: This module defines the ArmMotor class, which represents a single motor in the arm.


Author: Tyler Halifax
Date Created: Febuary 19, 2024
"""

"""
CONSTANTS:
Max. speed 2000RPM 
Rated speed 1500RPM

Position control mode Maximum input pulse frequency 500KHz

Pulse command mode: 
    Electronic gear ratio: set range from 1/65536: 1/65536 to 65536:1
    Position sampling frequency 2KHz

Communication interface Easycan (CAN communication, rate 1 M)
    

CANopen communication:
    Name            Canopen address     Access        Parameter
    Mode selection: 26190010            Read/Write    0-100
        0: Pulse + direction
        2: Encoder follows the pattern Speed mode, duty cycle speed regulation
        6: battery state, power automaticaly go back to multi-turn zero
        7: with brake motor, brake multi-turn absolute value funtion, power on multi-turn position update in the absolute position register

    Format:
        SOF: 1 bit -> interframe space
        Identifier: 11 bit -> 0 to 255 
        RTR: 1 bit -> 0: data frame(must be used for EasyCan), 1: remote frame
        r1:  1 bit -> 0: standard frame(must be used for EasyCan), 1: extended frame
        r0:  1 bit -> receive location
        DLC: 4 bit -> Data length code
        Data Segment: 0-64 bits
        CRC: 16 bit
        ACK: 2 bit
        EOF: 7 bit
        IFS: 3 bit


SDO communication:

    Name                            Canopen address     Access        Parameter
    Motor actual speed (RPM*10):    606c0010            Read          -30000-30000                       


    SDO Speed mode:
        Address     Name        Access   Description
        60FF0020    Mode speed  W        Set the speed mode

    
    
    
"""
"""
import openCAN;

class ArmMotor:
    def __init__(self, can_interface: openCAN.CANInterface, motor_id: int):
        self.can_interface = can_interface
        self.motor_id = motor_id

    def send_position(self, position: int):
        # Construct a CAN message with the motor ID and position
        message = openCAN.CANMessage(self.motor_id, position)
        # Send the message using the CAN interface
        self.can_interface.send_message(message)

    def get_position(self) -> int:
        # Request the current position from the motor
        self.can_interface.send_message(openCAN.CANMessage(self.motor_id, 'GET_POSITION'))
        # Wait for a response
        response = self.can_interface.receive_message()
        # Return the position from the response
        return response.data


"""
    