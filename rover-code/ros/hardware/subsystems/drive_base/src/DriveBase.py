#!/usr/bin/env python3
""" 
File: DriveBase.py 
 
Description: This file contains the implementation of a DriveBase class that 
extends the DifferentialDrive class. It provides methods for controlling the 
movement of a rover using drive wheels. 
 
Author: Ryan Barry
Date Created: August 12, 2023
"""

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import rospkg
import sys

rospack = rospkg.RosPack()
package_path = rospack.get_path('communications')
sys.path.append(package_path + '/src')
package_path = rospack.get_path('constants')
sys.path.append(package_path + '/src')



from MobileRobotKinematics import MobileRobotKinematics
from RoverConstants import WHEEL_NAMES
from RoverPinout import *
from CAN_Constants import TOPICS, get_id
from communications.msg import CAN_msg
        

TARGET_VELOCITY_INDEX = get_id("TARGET_VELOCITY")
CURRENT_VELOCITY_INDEX = get_id("CURRENT_VELOCITY")

MAX_WHEEL_VEL = 32 # RPM
        

class DriveBase(MobileRobotKinematics):
    def __init__(self):
        MobileRobotKinematics.__init__(self)

        rospy.init_node("drive_base_node")
        rospy.Subscriber(f"/CAN/RX/{TOPICS[CURRENT_VELOCITY_INDEX]['name']}", CAN_msg, self.current_velocity_callback)
        rospy.Subscriber("/drive_base/cmd_vel", Twist, self.target_velocity_callback)
        self.target_velo_pub = rospy.Publisher(f"/CAN/TX/{TOPICS[TARGET_VELOCITY_INDEX]['name']}", CAN_msg, queue_size=10)
        self.current_velo_pub = rospy.Publisher("/drive_base/current_velocity", Twist, queue_size=10)

        self.new_velocity = False

        self.run()
        

    def current_velocity_callback(self, msg: CAN_msg):
        velo = []
        rospy.loginfo("balls")
        for i in range(len(WHEEL_NAMES)):
            sign = -1 if ((msg.buf[len(WHEEL_NAMES)]) >> i) & 1 else 1
            velo.append(sign * self.decoder(msg.buf[i]))
        self.current_velocity = self.calculate_robot_velocity(velo)
        vel_msg = Twist()
        vel_msg.linear.x = self.current_velocity[0]
        vel_msg.linear.y = self.current_velocity[1]
        vel_msg.angular.z = self.current_velocity[2]
        self.current_velo_pub.publish(vel_msg)
        
        

    def target_velocity_callback(self, msg: Twist):
        target_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.calculate_wheel_velocities(target_vel)

        self.new_velocity = True

    def decoder(self, scaled_value):
        return scaled_value / 255 * MAX_WHEEL_VEL

    def encoder(self, velo):
        #do our scaling here
        scaler = min(abs(velo / MAX_WHEEL_VEL), 1)
        return int(255*scaler)

    def run(self):
        while not rospy.is_shutdown():
            if self.new_velocity:
                velo_can_msg = CAN_msg()
                velo_can_msg.buf = bytearray(velo_can_msg.buf)
                velo_can_msg.id = TOPICS[TARGET_VELOCITY_INDEX]['id']
                phi_list = self._phi.T.tolist()
                for i, velo in enumerate(phi_list):
                    velo_can_msg.buf[i] = self.encoder(velo)
                    if velo >= 0:
                        velo_can_msg.buf[6] &= ~(1 << i)
                    else:
                        velo_can_msg.buf[6] |= (1 << i)


                self.target_velo_pub.publish(velo_can_msg)
                self.new_velocity = False

if __name__ == "__main__":
    drive_base = DriveBase()
