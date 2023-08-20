#!/usr/bin/env python3
"""
File: RoverStatus.py

Description: This module defines the RoverStatus class, which represents the status of a rover in a ROS2 system. 
It provides functionality to subscribe to various topics and update the rover's operating mode, communication link 
status, and waypoint status based on received messages.

Author: Ryan Barry
Date created: June 26, 2023

Dependencies:

    rclpy: ROS2 Python client library
    rclpy.node: Node class for creating ROS2 nodes
    std_msgs.msg.Int32: ROS2 message type for representing integer values
    hardware.RoverConstants: Constants related to the rover's hardware configuration and states

"""
import rclpy
from hardware.RoverConstants import *
from rclpy.node import Node
from std_msgs.msg import Int32


class RoverStatus(Node):
    def __init__(self):
        super().__init__("rover_status")

        self._operating_mode = DRIVER_CONTROL_MODE
        self._comm_link_status = NOT_CONNECTED
        self._waypoint_status = WAYPOINT_INACTIVE

        # Create subscribers
        self.__operating_mode_subscriber = self._create_subscription(
            Int32, "operating_mode_topic", self._operating_mode_callback, 10
        )
        self.__comm_link_status_subscriber = self._create_subscription(
            Int32, "comm_link_status_topic", self._comm_link_status_callback, 10
        )
        self.__waypoint_status_subscriber = self._create_subscription(
            Int32, "waypoint_status_topic", self._waypoint_status_callback, 10
        )

    def operating_mode_callback(self, msg):
        self._operating_mode = msg.data

    def get_operating_mode(self):
        return self._operating_mode

    def comm_link_status_callback(self, msg):
        self._comm_link_status = msg.data

    def get_comm_link_status(self):
        return self._comm_link_status

    def waypoint_status_callback(self, msg):
        self._waypoint_status = msg.data

    def get_waypoint_status(self):
        return self._waypoint_status
