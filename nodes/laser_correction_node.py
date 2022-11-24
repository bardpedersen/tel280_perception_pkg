#!/usr/bin/env python3
import rospy
import numpy as np
from tel280_perception_pkg.helper import SimpleNavHelpers
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math


class LaserNode():

    def __init__(self):
        
        self.pcd_subscriber = rospy.Subscriber(
            "/scan", LaserScan,
            callback=self.listener_callback,
            queue_size=1,
            buff_size=2002428800)
        
        self.first_msg = LaserScan()
        self.first_msg_recieved = False
        
        # 4 publishing laser scan projected image
        self.corrected_scan_pub = rospy.Publisher("/corrected_scan", LaserScan)
        
    def listener_callback(self, msg: LaserScan):
        rospy.loginfo(str("Point cloud recieved"))
        
        if not self.first_msg_recieved:
            self.first_msg = msg
            self.first_msg_recieved = True
            
            self.first_msg.ranges = list(self.first_msg.ranges)
            self.first_msg.intensities = list(self.first_msg.intensities)
            self.first_msg.ranges.append(self.first_msg.ranges[0])
            self.first_msg.intensities.append(self.first_msg.intensities[0])
            self.first_msg.ranges = tuple(self.first_msg.ranges)
            self.first_msg.intensities = tuple(self.first_msg.intensities)    
        
        msg.angle_increment = self.first_msg.angle_increment
        msg.angle_min = self.first_msg.angle_min
        msg.angle_max = self.first_msg.angle_max
        
        diff = len(msg.ranges) - len(self.first_msg.ranges)
                
        print(len(self.first_msg.ranges))
            
        if diff > 0:
            msg.ranges = msg.ranges[:len(msg.ranges)-(diff)]
            msg.intensities = msg.intensities[:len(msg.intensities)-(diff)]

        if diff < 0:
            for i in range(0,abs(diff)):
                msg.ranges = list(msg.ranges)
                msg.intensities = list(msg.intensities)
                msg.ranges.append(msg.ranges[0])
                msg.intensities.append(msg.intensities[0])
                msg.ranges = tuple(msg.ranges)
                msg.intensities = tuple(msg.intensities)    

                          
        self.corrected_scan_pub.publish(msg)        

def main():
    rospy.init_node("LaserNode")
    node = LaserNode()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
