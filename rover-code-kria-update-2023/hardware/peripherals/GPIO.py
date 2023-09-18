# Import the PYNQ library and the base overlay
"""
File: GPIO.py

Description: This file defines a GPIO class that be used for RPiHat or PMod headers.
The Kria KR260 uses the Petalinux framework for GPIO regulation. PYNQ includes a base 
overlay for the KR260 that handles petalinux natively in Python enabling script-based
GPIO control.

Author: Ryan Barry
Date Created: September 18, 2023
"""
import pynq
from pynq.lib import RPi

# Load the base overlay
base = pynq.Overlay("base.bit")

# Create an instance of the RPi interface
rpi = base.rpi

# Define a class for GPIO pins
class GPIO:
    # Initialize the GPIO object with a pin number and a direction
    def __init__(self, pin, direction):
        # Set the pin number
        self.pin = pin
        # Set the pin direction ("in" or "out")
        self.direction = direction
        # Set the pin mode using the RPi interface
        rpi.set_pin(pin, "gpio")
        # Set the pin direction using the RPi interface
        rpi.set_direction(pin, direction)
    
    # Write a value (0 or 1) to the GPIO pin
    def write(self, value):
        # Check if the pin direction is "out"
        if self.direction == "out":
            # Write the value using the RPi interface
            rpi.write(self.pin, value)
        else:
            # Raise an exception if the pin direction is not "out"
            raise Exception(f"Cannot write to GPIO pin {self.pin} with direction {self.direction}")
    
    # Read a value (0 or 1) from the GPIO pin
    def read(self):
        # Check if the pin direction is "in"
        if self.direction == "in":
            # Read the value using the RPi interface
            value = rpi.read(self.pin)
            # Return the value
            return value
        else:
            # Raise an exception if the pin direction is not "in"
            raise Exception(f"Cannot read from GPIO pin {self.pin} with direction {self.direction}")
