import sys
sys.path.append("../..")
from RoverConstants import *
from peripherals.LED import LED

class CommLinkLED(LED):
    def __init__(self, comm_link_status):
        self.comm_link_status = comm_link_status
        
    def update(self):
        if self.comm_link_status == CONNECTED:
            pass
        elif self.comm_link_status == NOT_CONNECTED:
            pass
        
        
class OperatingModeLED(LED):
    def __init__(self, operating_mode):
        self.operating_mode = operating_mode
        
    def update(self):
        if self.operating_mode == DRIVER_CONTROL_MODE:
            self.set_color(BLUE)
        elif self.operating_mode == AUTONOMOUS_MODE:
            self.set_color(RED)
            
            
class WaypointLED(LED):
    def __init__(self, waypoint_status):
        self.waypoint_status = waypoint_status
        
    def update(self):
        if self.waypoint_status == WAYPOINT_COMPLETE:
            self.set_color(GREEN)