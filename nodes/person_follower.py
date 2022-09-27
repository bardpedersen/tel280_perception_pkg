#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped, Twist
import numpy as np
from tel280_perception_pkg.helper import generate_points
from tel280_perception_pkg.helper import RANSAC
from tel280_perception_pkg.helper import SimpleNavHelpers
from tel280_perception_pkg.helper import PurePursuitController

import open3d as o3d
import ros_numpy


class PersonFollowerNode():

    def __init__(self):
        pass
        
    def listener_callback(self, msg):
        rospy.loginfo(str("Point cloud recieved"))

    
def main():
    
    rospy.init_node("PersonFollowerNode")
    node = PersonFollowerNode()
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
