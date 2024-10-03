#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import rospkg
import sys
from sensor_msgs.msg import Imu, MagneticField

rospack = rospkg.RosPack()
package_path = rospack.get_path('communications')
sys.path.append(package_path + '/src')
package_path = rospack.get_path('constants')
sys.path.append(package_path + '/src')

from CAN_Constants import TOPICS, get_id
from communications.msg import CAN_msg

IMU_INDEX = get_id("IMU")


class IMU_Handler:
    def __init__(self):
        rospy.init_node("IMU_node")
        rospy.Subscriber(f"/CAN/RX/{TOPICS[IMU_INDEX]['name']}", CAN_msg, self.imu_callback)
        self.imu_pub = rospy.Publisher("/imu/accelerometer", Imu, queue_size=10)
        self.mag_pub = rospy.Publisher("/imu/magnetometer", MagneticField, queue_size=10)
        self.heading_pub = rospy.Publisher("/imu/heading", Float32, queue_size=10)
        
        self.magnetometer = MagneticField()
        rospy.spin()
        
    def imu_callback(self, msg: CAN_msg):
        """
        ax
        ay
        az
        mx
        my
        mz
        """
        self.decode_acc(msg.buf[:3])
        self.decode_mag(msg.buf[3:6])
        
    def decode_acc(self, accel):
        """
        Decode the acceleration bytes from the can message here
        """
        imu = Imu()
        imu.angular_velocity.x = accel[0]
        imu.angular_velocity.y = accel[1]
        imu.angular_velocity.z = accel[2]
        self.imu_pub.publish(imu)
    
    def decode_mag(self, mag):
        """
        Decode the magnetometer bytes from the can message here
        """
        magnetometer = MagneticField()
        magnetometer.magnetic_field.x = mag[0]
        magnetometer.magnetic_field.y = mag[1]
        magnetometer.magnetic_field.z = mag[2]
        self.mag_pub.publish(magnetometer)
        self.get_heading(magnetometer)
        
    def get_heading(self, mag: MagneticField, degrees=True):
        heading = np.arctan2(mag.magnetic_field.x, mag.magnetic_field.y) % (2*np.pi)
        if degrees:
            heading = np.degrees(heading)
        self.heading_pub.publish(heading)
            
            
        
        
        
        
if __name__ == "__main__":
    imu = IMU_Handler()