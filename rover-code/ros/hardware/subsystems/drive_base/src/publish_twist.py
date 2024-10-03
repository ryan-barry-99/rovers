#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def publish_twist():
    rospy.init_node('twist_publisher', anonymous=True)
    pub = rospy.Publisher('/drive_base/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 1.0

        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_twist()
    except rospy.ROSInterruptException:
        pass
