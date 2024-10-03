#!/usr/bin/env python3
import rospkg
import sys
rospack = rospkg.RosPack()
package_path = rospack.get_path('constants')
sys.path.append(package_path + '/src')

import rospy
from communications.msg import CAN_msg
from CAN_Constants import CHANNEL, TOPICS, get_id

# published = False

class CAN_Test:
    def __init__(self, mode, channel, message_id, buf):
        rospy.init_node("CAN_Testing_Node")
        global published
        
        message_id = get_id(message_id)
        self.pub = rospy.Publisher(f"/CAN/{mode}/{TOPICS[message_id]['name']}", CAN_msg, queue_size=10)
        msg = CAN_msg()
        msg.channel = CHANNEL[channel]
        msg.id = message_id
        msg.buf = buf
        # if not published:
        
        self.pub.publish(msg)
        rospy.loginfo(f"Publishing to /CAN/{mode}/{TOPICS[message_id]['name']}")
        rospy.loginfo(msg)
        # published = True




CH = "MAIN_BODY"
MODE = "RX"
MESSAGE_ID = get_id("E_STOP")
BUF = [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

rospy.init_node("CAN_Testing_Node")

pub = rospy.Publisher(f"/CAN/{MODE}/{TOPICS[MESSAGE_ID]['name']}", CAN_msg, queue_size=10)
msg = CAN_msg()
msg.channel = CHANNEL[CH]
msg.id = MESSAGE_ID
msg.buf = BUF
def published(msg):
    rospy.loginfo(f"received message {msg}")
    global stop
    stop = True
rospy.Subscriber(f"/CAN/{MODE}/{TOPICS[MESSAGE_ID]['name']}", CAN_msg, published)
stop = False

  
if __name__ == "__main__":
    while not rospy.is_shutdown() and not stop:
        
        pub.publish(msg)
        rospy.loginfo(f"Publishing to /CAN/{MODE}/{TOPICS[MESSAGE_ID]['name']}")
        rospy.loginfo(msg)
            
            
        
    
    