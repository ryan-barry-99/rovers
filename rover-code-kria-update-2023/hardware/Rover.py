#!/usr/bin/env python3
"""
File: Rover.py

Description:This file defines the "Rover" class, which represents 
the main hardware and functionalities of the rover. It acts as an 
interface between the rover's missions and its various subsystems. 
The class handles the initialization of the rover's status, peripherals, 
and subsystems, as well as updating the LEDs based on the rover's status.

Author: Ryan Barry
Date Created: July 16, 2023
"""

import sys

import rclpy

sys.path.append("..")
from communications.Communications import Communications
from ErrorHandler import ErrorHandler
from hardware.status.RoverStatus import RoverStatus
from hardware.status.StatusLEDs import CommLinkLED, OperatingModeLED, WaypointLED
from rclpy.node import Node
from RoverConstants import *
from RoverPinout import *
from sensors.Camera import Camera
from sensors.LiDAR import LiDAR
from subsystems.arm.ArmRobot import ArmRobot
from subsystems.drive_base.DriveBase import DriveBase
from subsystems.science_plate.SciencePlate import SciencePlate
from subsystems.cameras.FrontCamera import FrontCamera


class Rover(Node, ErrorHandler):
    def __init__(self):
        ErrorHandler.__init__(self)
        Node.__init__("rover")
        rclpy.spin(self)

        # Initialize Status
        self.status = RoverStatus()
        self.__active_mission = None

        # Initialize Peripherals
        self.arm = ArmRobot()
        self.science_plate = SciencePlate()
        self.operating_mode_LED = OperatingModeLED(self.status.operating_mode)
        self.comm_link_LED = CommLinkLED(self.status.comm_link_status)
        self.waypoint_LED = WaypointLED(self.status.waypoint_status)
        self.lidar = LiDAR()
        self.front_cam = FrontCamera()
        self.drive_base = DriveBase(self.status.operating_mode)
        self.comms = Communications()

    def get_mission(self):
        missions = [EXTREME_RETRIEVAL_DELIVERY, SCIENCE, AUTONOMOUS, EQUIPMENT_SERVICING]
        if self.__active_mission in missions:
            return self.__active_mission
        self.log_error(MISSION_FAILURE)

    def set_mission(self, mission):
        self.__active_mission = mission

    def update_LEDs(self):
        self.operating_mode_LED.update()
        self.comm_link_LED.update()
        self.waypoint_LED.update()

    def run(self):
        rclpy.spin(self.arm)  # Spin the ArmRobot node

        self.status.destroy_node()
        self.arm.destroy_node()
