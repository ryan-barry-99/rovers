""" 
File: DriveBase.py 
 
Description: This file contains the implementation of a DriveBase class that 
extends the DifferentialDrive class. It provides methods for controlling the 
movement of a rover using drive wheels. 
 
Author: Ryan Barry
Date Created: August 12, 2023
"""
import sys

sys.path.append("../..")
from DifferentialDrive import DifferentialDrive
from DriveWheel import DriveWheel
from RoverConstants import WHEEL_NAMES
from RoverPinout import *


class DriveBase(DifferentialDrive):
    def __init__(self):
        DifferentialDrive.__init__(self)
        self.left_wheels = []
        self.right_wheels = []

        for i in range(len(WHEEL_NAMES)):
            name = WHEEL_NAMES[i]
            pwm_pin = WHEEL_PINS[f"{name}_pwm"]
            wheel = DriveWheel(name=name, gpio_pin=pwm_pin)
            
            if "left" in name:
                self.left_wheels.append(wheel)
            elif "right" in name:
                self.right_wheels.append(wheel)

