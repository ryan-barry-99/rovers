import rclpy
from rclpy.node import Node
import sys
sys.path.append('../hardware/')
from hardware.RoverConstants import *
from std_msgs.msg import Int32

class Communications(Node):
    def __init__(self):
        super().__init__("communication_antenna")

        self.waypoint_status = WAYPOINT_INACTIVE
        self.comm_link_status = NOT_CONNECTED

        self.waypoint_status_subscriber = self.create_subscription(Int32, "waypoint_status", self.waypoint_status_callback, 10)


    def waypoint_status_callback(self, msg):
        self.waypoint_status = msg