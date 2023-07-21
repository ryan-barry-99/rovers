import rclpy
from rclpy.node import Node

class LiDAR(Node):
    def __init__(self,name=1):
        super().__init__(f"lidar{name}")