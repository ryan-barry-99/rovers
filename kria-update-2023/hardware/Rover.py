import rclpy
from RoverConstants import *
from hardware.RoverStatus import RoverStatus
from rover_subsystems.ArmRobot import ArmRobot
from rover_subsystems.Camera import Camera
from rover_subsystems.LED import LED

class Rover:
    def __init__(self):
        # Initialize Status
        self.status = RoverStatus()
        self.active_mission = EXTREME_RETRIEVAL_DELIVERY

        # Initialize Peripherals
        self.arm = ArmRobot()
        self.operating_mode_LED = LED(color=BLUE)
        self.comm_link_LED = LED()
        self.waypoint_LED = LED()

    def update_LEDs(self):
        if self.status.operating_mode == DRIVER_CONTROL_MODE:
            self.operating_mode_LED.set_color(BLUE)
        elif self.status.operating_mode == AUTONOMOUS_MODE:
            self.operating_mode_LED.set_color(RED)
        # Add code to set the operating mode LED color

        if self.status.comm_link_status == CONNECTED:
            pass
        elif self.status.comm_link_status == NOT_CONNECTED:
            pass

        if self.status.waypoint_status == WAYPOINT_COMPLETE:
            self.waypoint_LED.set_color(GREEN)
            # Add code to set waypoint LED

    def run(self):
        rclpy.spin(self.status)  # Spin the RoverStatus node
        rclpy.spin(self.arm)  # Spin the ArmRobot node

        self.status.destroy_node()
        self.arm.destroy_node()
