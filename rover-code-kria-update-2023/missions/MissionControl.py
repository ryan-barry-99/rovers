#!/usr/bin/env python3
"""
File: mission_control.py

Description: This script serves as the main control module for a rover system. 
It utilizes the Robot Operating System (ROS) client library (rclpy) to interface 
with the rover's hardware components. The program defines a MissionControl class 
responsible for managing various missions that the rover can execute. Each mission 
is represented by a separate class: AutonomousNavigationMission, 
EquipmentServicingMission, ExtremeRetrievalDeliveryMission, and ScienceMission.

Author: Ryan Barry
Date Created: July 16, 2023
"""

import rclpy  # Importing the rclpy module for ROS 2 Python client library
from hardware.RoverConstants import AUTONOMOUS, EQUIPMENT_SERVICING, EXTREME_RETRIEVAL_DELIVERY, SCIENCE  # Importing missiion constants from the RoverConstants module
from AutonomousNavigation import AutonomousNavigationMission  # Importing the AutonomousNavigationMission class
from EquipmentServicing import EquipmentServicingMission  # Importing the EquipmentServicingMission class
from ExtremeRetrievalDelivery import ExtremeRetrievalDeliveryMission  # Importing the ExtremeRetrievalDeliveryMission class
from ScienceMission import ScienceMission  # Importing the ScienceMission class
import sys  # Importing the sys module for system-specific parameters and functions

sys.path.append("..")  # Adding the parent directory to the system path

from communications.XBox_Controller import XboxController  # Importing the XboxController class from the specified module
from hardware.Rover import Rover  # Importing the Rover class from the specified module

class MissionControl:  # Defining a class named MissionControl
    def __init__(self):  # Constructor for the MissionControl class
        self.rover = Rover()  # Creating an instance of the Rover class and assigning it to the rover attribute
        self.autonomous_navigation = AutonomousNavigationMission(self.rover)  # Creating an instance of the AutonomousNavigationMission class and assigning it to the autonomous_navigation attribute
        self.equipment_servicing = EquipmentServicingMission(self.rover)  # Creating an instance of the EquipmentServicingMission class and assigning it to the equipment_servicing attribute
        self.extreme_retrieval_delivery = ExtremeRetrievalDeliveryMission(self.rover)  # Creating an instance of the ExtremeRetrievalDeliveryMission class and assigning it to the extreme_retrieval_delivery attribute
        self.science_mission = ScienceMission(self.rover)  # Creating an instance of the ScienceMission class and assigning it to the science_mission attribute
        self.controller = XboxController()  # Creating an instance of the XboxController class and assigning it to the controller attribute

    def exec(self):  # Method for executing the mission control logic
        self.controller.publish_controller_state()  # Publishing the controller state
        mission = self.rover.get_mission()  # Getting the current mission from the rover
        if mission == AUTONOMOUS:  # Checking if the mission is AUTONOMOUS
            self.autonomous_navigation.run()  # Running the autonomous navigation mission
        if mission == EQUIPMENT_SERVICING:  # Checking if the mission is EQUIPMENT_SERVICING
            self.equipment_servicing.run()  # Running the equipment servicing mission
        if mission == EXTREME_RETRIEVAL_DELIVERY:  # Checking if the mission is EXTREME_RETRIEVAL_DELIVERY
            self.extreme_retrieval_delivery.run()  # Running the extreme retrieval and delivery mission
        if mission == SCIENCE:  # Checking if the mission is SCIENCE
            self.science_mission.run()  # Running the science mission
