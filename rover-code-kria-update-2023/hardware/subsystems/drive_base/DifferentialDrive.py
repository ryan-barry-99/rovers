import sys
sys.path.append('../..')

from RoverConstants import WHEEL_RADIUS, BASE_WIDTH
import math

class DifferentialDrive:
    def __init__(self, wheel_radius=WHEEL_RADIUS, wheel_base=BASE_WIDTH):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base

    def forward_kinematics(self, left_wheel_velocity, right_wheel_velocity):
        linear_velocity = (self.wheel_radius / 2) * (left_wheel_velocity + right_wheel_velocity)
        angular_velocity = (self.wheel_radius / self.wheel_base) * (right_wheel_velocity - left_wheel_velocity)
        return linear_velocity, angular_velocity

    def inverse_kinematics(self, linear_velocity, angular_velocity):
        left_wheel_velocity = (2 * linear_velocity - angular_velocity * self.wheel_base) / (2 * self.wheel_radius)
        right_wheel_velocity = (2 * linear_velocity + angular_velocity * self.wheel_base) / (2 * self.wheel_radius)
        return left_wheel_velocity, right_wheel_velocity