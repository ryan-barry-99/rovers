#!/usr/bin/env python3
# from pydoc_data import topics
import rospkg
import sys
rospack = rospkg.RosPack()
package_path = rospack.get_path('constants')
sys.path.append(package_path + '/src')

import rospy
from communications.msg import CAN_msg, CAN_FD_msg
from CAN_Constants import CHANNEL, TOPICS
import serial

class CAN_Interface:
    def __init__(self):
        rospy.init_node("can_interface")
        
        # J44 Header TX and RX
        self.ser = serial.Serial(
            port="/dev/ttyS0",
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        
        self.pubs = {}
        for topic in TOPICS:
            self.pubs[TOPICS[topic]["id"]] = rospy.Publisher(f"/CAN/RX/{TOPICS[topic]['name']}", CAN_msg, queue_size=10)
            rospy.Subscriber(f"/CAN/TX/{TOPICS[topic]['name']}", CAN_msg, self.send_msg)
          
        self.run()
        
          
    def send_msg(self, msg):
        message = f"{msg.channel}/{msg.id}/{msg.buf}\n"
        self.ser.write(message.encode('utf-8'))

    def run(self):
        while not rospy.is_shutdown():
            msg = self.ser.readline().split('/')
            new_message = CAN_msg()
            new_message.channel = msg[0]
            new_message.id = msg[1]
            new_message.buf = msg[2]
            
            self.pubs[new_message.id].publish(new_message)
        self.ser.close()
            
            
            
if __name__ == "__main__":
    can = CAN_Interface()