#!/usr/bin/env python3
import rospy
import math
import sys
import numpy as np
from nav_msgs.msg import Odometry


class Perception:

    def __init__(self):
        def __init__(self):
        self.source_topic = "odom"
        self.published_topic = "odom_noisy"
        self.position_noise = 0.08
        self.velocity_noise = 0.08
        self.percepted_odom = Odometry()
        rospy.Subscriber(self.source_topic, Odometry, self.source_callback)
        self.pub = rospy.Publisher(
            self.published_topic, Odometry, queue_size=10)
        
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.pub.publish(self.percepted_odom)
            rate.sleep()
            

    def source_callback(self, odom):
        self.percepted_odom = odom

        self.percepted_odom.pose.pose.position.x = odom.pose.pose.position.x + \
            np.random.normal(0, self.position_noise)
        self.percepted_odom.pose.pose.position.y = odom.pose.pose.position.y + \
            np.random.normal(0, self.position_noise)
        self.percepted_odom.twist.twist.linear.x = odom.twist.twist.linear.x + \
            np.random.normal(0, self.velocity_noise)
        self.percepted_odom.twist.twist.angular.z = odom.twist.twist.angular.z + \
            np.random.normal(0, self.velocity_noise)
        self.pub.publish(self.percepted_odom)


if __name__ == '__main__':
    rospy.init_node("Perception")
    try:
        node = Perception()
    except rospy.ROSInterruptException:
        pass
