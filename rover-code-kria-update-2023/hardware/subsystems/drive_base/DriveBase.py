import sys

sys.path.append("../peripherals/")
from peripherals.Motor import Motor


class DriveBase:
    def __init__(self):
        self.fLeft = Motor()
        self.mLeft = Motor()
        self.bLeft = Motor()

        self.fRight = Motor()
        self.mRight = Motor()
        self.bRight = Motor()
