from time import sleep
import can
import rospy
from typing import Optional
from ctypes import *


class CAN_MBB:
    def __init__(self) -> None:
        rospy.init_node("CAN_MBB")

        rospy.Subscriber("/main_body_board", c_uint32, self.callback)

        self.pub = rospy.Publisher("/main_body_board_can", c_uint32)

    def callback(self, data: c_uint32) -> None:
        self.send(data)


class CAN:
    poll_rate: int  # milliseconds

    def __init__(self) -> None:
        rospy.init_node("CAN")

        self.poll_rate = 10
        self.pub = rospy.Publisher("/main")

        self.run()

    def send(data: c_uint32) -> bool:
        with can.Bus() as bus:
            msg = can.Message(arbitration_id=0xC0FFEE, data=data, is_extended_id=True)

            try:
                bus.send(msg)
                # print(f"Message sent on {bus.channel_info}")
                return True
            except:
                # print("Message not sent")
                return False

    def receive() -> Optional[c_uint32]:
        with can.Bus() as bus:
            msg = bus.recv()

            if msg is not None:
                return msg.data
            return None

    def run(self):
        while not rospy.is_shutdown():
            msg = self.receive()

            if msg is not None:
                self.pub.publish(msg)

            sleep(self.poll_rate)
