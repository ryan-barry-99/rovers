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

import rclpy
from rclpy.node import Node

from communications.Communications import Communications
from hardware.status.RoverStatus import RoverStatus
from hardware.status.StatusLEDs import CommLinkLED, OperatingModeLED, WaypointLED
from RoverConstants import *
from sensors.Camera import Camera
from sensors.LiDAR import LiDAR
from subsystems.arm.ArmRobot import ArmRobot
from subsystems.drive_base.DriveBase import DriveBase
from subsystems.science_plate.SciencePlate import SciencePlate


class Rover(Node):
    def __init__(self):
        super().__init__("rover")

        # Initialize Status
        self.status = RoverStatus()
        self.__active_mission = EXTREME_RETRIEVAL_DELIVERY

        # Initialize Peripherals
        self.arm = ArmRobot()
        self.science_plate = SciencePlate()
        self.operating_mode_LED = OperatingModeLED(self.status.operating_mode)
        self.comm_link_LED = CommLinkLED(self.status.comm_link_status)
        self.waypoint_LED = WaypointLED(self.status.waypoint_status)
        self.lidar = LiDAR()
        self.front_cam = Camera(name="front_camera", camera_index=0)
        self.drive_base = DriveBase()
        self.comms = Communications()

    def get_mission(self):
        return self.__active_mission

    def set_mission(self, mission):
        self.__active_mission = mission

    def update_LEDs(self):
        self.operating_mode_LED.update()
        self.comm_link_LED.update()
        self.waypoint_LED.update()

    def run(self):
        rclpy.spin(self.status)  # Spin the RoverStatus node
        rclpy.spin(self.arm)  # Spin the ArmRobot node

        self.status.destroy_node()
        self.arm.destroy_node()
