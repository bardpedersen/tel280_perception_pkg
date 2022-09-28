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
        
        self.pcd_subscriber = rospy.Subscriber(
            "/camera/depth/points", PointCloud2,
            callback=self.listener_callback,
            queue_size=1,
            buff_size=2002428800)

        self.obstacle_pcd_publisher = rospy.Publisher(
            "obstacle_pcd", PointCloud2, queue_size=1)

        self.obstacle_box_pub = rospy.Publisher(
            "obstacle_box", Marker, queue_size=1)

        self.follow_person_cmd = rospy.Publisher(
            "cmd_vel", Twist, queue_size=1)

        self.navigation_helpers = SimpleNavHelpers()
        self.controller = PurePursuitController(
            linear_k=20, angular_k=5, linear_max=0.20, angular_max=0.1)

        self.latest_goal_pose = PoseStamped()
        
    def listener_callback(self, msg):
        rospy.loginfo(str("Point cloud recieved"))

        # Convert from msg.PointCloud2 to Open3D PointCloud
        pcd_as_numpy_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(
            msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcd_as_numpy_array)

        # Reduce the density of the cloud
        # downpcd = pcd.uniform_down_sample(every_k_points=2)
        downpcd = pcd.voxel_down_sample(voxel_size=0.05)        

        # Revove the ground plane , to identify the objects
        plane_model, inliers = downpcd.segment_plane(distance_threshold=0.05,
                                                     ransac_n=3,
                                                     num_iterations=100)
        # This is still a Open3D point cloud
        inlier_cloud = downpcd.select_by_index(inliers)
        outlier_cloud = downpcd.select_by_index(
            inliers, invert=True)  # This is still a Open3D point cloud
        
        # If point depicting human are less than 10 , do nothing, it might be just noise
        if len(outlier_cloud.points) < 10:
            return

        bbx = outlier_cloud.get_axis_aligned_bounding_box()
        corners = bbx.get_box_points()

        marker = Marker()
        marker.header = msg.header
        marker.id = 0
        marker.ns = "marker"
        marker.action = Marker().ADD
        marker.type = Marker().LINE_STRIP
        marker.scale.x = 0.05
        marker.color.g = 1.0
        marker.color.a = 1.0
        for corner in corners:
            point = Point()
            point.x = corner[0]
            point.y = corner[1]
            point.z = corner[2]
            marker.points.append(point)
            marker.colors.append(marker.color)

        self.obstacle_box_pub.publish(marker)

        # However this is in camera frmae, we need to transfrom this
        # to odom frame
        bbx_center = bbx.get_center()
        target_point_in_camera_frame = Point(
            bbx_center[0], bbx_center[1], bbx_center[2])

        target_point_in_odom = self.navigation_helpers.transform_point(
            target_point_in_camera_frame, source_frame=msg.header.frame_id, target_frame="odom")

        curr_goal_pose = PoseStamped()
        curr_goal_pose.header.frame_id = "odom"
        curr_goal_pose.header.stamp = rospy.Time.now()
        curr_goal_pose.pose.position = target_point_in_odom
        curr_goal_pose.pose.orientation.w = 1.0

        self.latest_goal_pose = curr_goal_pose

        curr_robot_pose = self.navigation_helpers.get_curr_robot_pose()

        v, w = self.controller.compute_velocities(
            curr_robot_pose,  self.latest_goal_pose, dist_to_goal_satisfied=False)

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.follow_person_cmd.publish(twist)

        # Publish the obstacle cloud to RVIZ, THIS SLOWS DOWN PROGRAM
        obs_pcd = generate_points(
            np.asarray(outlier_cloud.points), msg.header.frame_id)

        obs_pcd.header.stamp = rospy.Time.now()
        self.obstacle_pcd_publisher.publish(obs_pcd)
    
def main():
    
    rospy.init_node("PersonFollowerNode")
    node = PersonFollowerNode()
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
