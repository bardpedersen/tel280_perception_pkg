#!/usr/bin/env python3
import rospy
import numpy as np
from tel280_perception_pkg.helper import SimpleNavHelpers
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math


class SensorFuserNode():

    def __init__(self):
        # Use ApproximateTimeSynchronizer to get synced sensor data
        pass

    def listener_callback(self, image: Image, camerainfo: CameraInfo, laser: LaserScan):

        rospy.loginfo(str("Recieved synced Image, CameraInfo, and LaserScan"))
        try:
            # convert sensor_msgs::Image -> opencv image
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            # Let us know if it was a failure
            print(e)


def main():

    rospy.init_node("SensorFuserNode")
    node = SensorFuserNode()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
