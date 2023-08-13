import sys
sys.path.append("../..")
from RoverPinout import *
from DriveWheel import DriveWheel
from DifferentialDrive import DifferentialDrive


class DriveBase(DifferentialDrive):
    def __init__(self):
        self.fLeft = DriveWheel(name="frontLeft", gpio_pin=FRONT_LEFT_PIN)
        self.mLeft = DriveWheel(name="middleLeft", gpio_pin=MIDDLE_LEFT_PIN)
        self.bLeft = DriveWheel(name="backLeft", gpio_pin=BACK_LEFT_PIN)

        self.fRight = DriveWheel(name="frontRight", gpio_pin=FRONT_RIGHT_PIN)
        self.mRight = DriveWheel(name="middleRight", gpio_pin=MIDDLE_RIGHT_PIN)
        self.bRight = DriveWheel(name="backRight", gpio_pin=BACK_RIGHT_PIN)
        
        self.left_velo = 0.0
        self.right_velo = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
    def get_velo(self):
        self.linear_velocity, self.angular_velocity = self.forward_kinematics(self.left_velo, self.right_velo)
