#!/usr/bin/env python3
"""
File: SciencePlate.py

Description:

Author: Ryan Barry
Date Created:
"""
import sys  # Importing the sys module for system-specific parameters and functions

import rclpy  # Importing the rclpy module for ROS 2 Python client library
from rclpy.node import Node  # Importing the Node class from rclpy.node module
from std_msgs.msg import Int32  # Importing the Int32 message type from the std_msgs package

sys.path.append("../..")  # Adding the parent directory to the system path

from RoverConstants import (
    DNA_DETECTED,  # Importing constants
    DNA_NOT_DETECTED,
    LIPID_DETECTED,
    LIPID_NOT_DETECTED,
)
from subsystems.cameras.ScienceCamera import (
    ScienceCamera,
)  # Importing the ScienceCamera class from the specified module


class SciencePlate(Node):  # Defining a class named SciencePlate that inherits from the Node class
    def __init__(self):  # Constructor for the SciencePlate class
        super().__init__(
            "science_plate"
        )  # Initializing the parent class Node with the name "science_plate"
        self.camera = (
            ScienceCamera()
        )  # Creating an instance of the ScienceCamera class and assigning it to the camera attribute
        self.lipid_publisher = self.create_publisher(
            Int32, "science/lipid_detection", 10
        )  # Creating a publisher for lipid detection messages
        self.dna_publisher = self.create_publisher(
            Int32, "science/dna_detection", 10
        )  # Creating a publisher for DNA detection messages
        self.lipid_detection = LIPID_NOT_DETECTED  # Initializing lipid_detection attribute with the constant value LIPID_NOT_DETECTED
        self.dna_detection = DNA_NOT_DETECTED  # Initializing dna_detection attribute with the constant value DNA_NOT_DETECTED
        rclpy.spin(self)  # Starting the ROS 2 event loop

    def publish_lipid_detection(self):  # Method for publishing lipid detection status
        lipid_detection_msg = Int32()  # Creating an instance of the Int32 message type
        lipid_detection_msg.data = (
            self.lipid_detection
        )  # Setting the data field of the message to the value of lipid_detection
        self.lipid_publisher.publish(lipid_detection_msg)  # Publishing the lipid detection message

    def publish_dna_detection(self):  # Method for publishing DNA detection status
        dna_detection_msg = Int32()  # Creating an instance of the Int32 message type
        dna_detection_msg.data = (
            self.dna_detection
        )  # Setting the data field of the message to the value of dna_detection
        self.dna_publisher.publish(dna_detection_msg)  # Publishing the DNA detection message

    def publish_detections(self):  # Method for publishing both lipid and DNA detection statuses
        self.publish_lipid_detection()  # Calling the publish_lipid_detection method
        self.publish_dna_detection()  # Calling the publish_dna_detection method
