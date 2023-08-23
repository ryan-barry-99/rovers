"""
File: FrontCamera.py

Description: This file contains attributes for the rover's front camera.
Any peripheral related to this camera (servos etc) should be instantiated here.

Author: Ryan Barry
Date Created: August 23, 2023
"""
import sys
sys.path.append("../..")
from sensors.Camera import Camera
from RoverPinout import FRONT_CAMERA_INDEX

class FrontCamera(Camera):
    def __init__(self, name="front_camera", camera_index=FRONT_CAMERA_INDEX):
        super().__init__(self, name, camera_index)

    def rotate(self):
        pass