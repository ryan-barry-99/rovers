'''
File: LED.py

This module contains a class enabling the 
status or color of an LED peripheral to be changed

Author: Ryan Barry
Date Created: July 16, 2023
'''

import sys
sys.path.append('../')
from RoverConstants import OFF, ON

class LED:
    def __init__(self, status=OFF, rgb=False, color=None):
        self.status = status
        self.rgb = rgb
        if color is not None:
            self.rgb = True
            self.set_color(color)

    def on(self):
        self.status = ON

    def off(self):
        self.status = OFF

    def set_color(self, color):
        if self.rgb:
            self.status = color