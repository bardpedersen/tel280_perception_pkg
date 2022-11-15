#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseArray

# Create A action client to move_base action server
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from transforms3d.euler import quat2euler
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Bool 


class RowFollower:
    def __init__(self):
        self.action_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                         self.initialpose_callback, queue_size=1)
        
        rospy.Subscriber("/stop", Bool,
                         self.stop_callback, queue_size=1)
        
        rospy.loginfo("We have connected to move_base action server")
        
        self.row_length = rospy.get_param('~row_length', 2.0)
        self.row_seperation_dist = rospy.get_param('~row_seperation_dist', 0.3)
        self.num_rows = rospy.get_param('~num_rows', 4)
        self.begin_from_left_row = rospy.get_param(
            '~begin_from_left_row', True)  # m
        
        self.waypoints_publisher = rospy.Publisher(
            "/waypoints", MarkerArray, queue_size=1)
        
        self.stop = Bool()
        self.stop.data = False

    def stop_callback(self, msg: Bool):
        rospy.loginfo(" Recieved cancel req.")
        self.stop = msg
        
    def deep_copy_pose(self, pose):
        dp = Pose()
        dp.position.x = pose.position.x
        dp.position.y = pose.position.y
        dp.orientation.x = pose.orientation.x
        dp.orientation.y = pose.orientation.y
        dp.orientation.z = pose.orientation.z
        dp.orientation.w = pose.orientation.w   
        return dp
             
    def initialpose_callback(self, initial_pose: PoseWithCovarianceStamped):
 
        waypoints = []
        marker_array = MarkerArray()
        
        marker_id = 0
        
        if self.begin_from_left_row == False:
            self.row_seperation_dist = -self.row_seperation_dist
        
        waypoints.append(self.deep_copy_pose(initial_pose.pose.pose))
        
        for i in range(0, self.num_rows):
            
            latest_waypoint = self.deep_copy_pose(waypoints[-1])

            latest_quat = [latest_waypoint.orientation.w, latest_waypoint.orientation.x,
                            latest_waypoint.orientation.y, latest_waypoint.orientation.z]
            latest_euler = quat2euler(latest_quat)
            
            heading_to_row = 1.57
            if(i % 2 == 1):
                heading_to_row = -1.57
            
            latest_waypoint.position.x += math.cos(latest_euler[2]) * self.row_length
            latest_waypoint.position.y += math.sin(latest_euler[2]) * self.row_length
            latest_waypoint.orientation = Quaternion(*quaternion_from_euler(0, 0, latest_euler[2]))
            waypoints.append(self.deep_copy_pose(latest_waypoint))

            marker = self.get_marker()
            marker.pose = latest_waypoint
            marker.id = marker_id
            marker_id += 1
            marker_array.markers.append(marker)

            latest_waypoint = self.deep_copy_pose(waypoints[-1])
            latest_waypoint.position.x += math.cos(latest_euler[2] + heading_to_row) * self.row_seperation_dist
            latest_waypoint.position.y += math.sin(latest_euler[2] + heading_to_row) * self.row_seperation_dist
            latest_waypoint.orientation = Quaternion(*quaternion_from_euler(0, 0, -latest_euler[2]))
            waypoints.append(self.deep_copy_pose(latest_waypoint))

            marker = self.get_marker()
            marker.pose = latest_waypoint
            marker.id = marker_id
            marker_id += 1
            marker_array.markers.append(marker)
            
        rospy.loginfo("We have Waypoints %d", len(waypoints))
        print(waypoints)

        self.waypoints_publisher.publish(marker_array)

        self.start_waypoint_following(marker_array)

    def get_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker().ARROW
        marker.action = Marker().ADD
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.ns = "wps"
        return marker


    def feedback_callback(self, feedback):
        rospy.loginfo("The Feedback is %s" % str(feedback))

    def start_waypoint_following(self, waypoint_markers: MarkerArray):
        
        i = 0
        for marker in waypoint_markers.markers:
            
            rospy.loginfo("Processing %d th waypoint", i)
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "odom"
            goal.target_pose.header.stamp = rospy.Time().now()
            goal.target_pose.pose = marker.pose 

            self.action_client.send_goal(goal, feedback_cb=self.feedback_callback)

            rate = rospy.Rate(1)
            while self.action_client.get_state() != GoalStatus().SUCCEEDED:
                rospy.loginfo("The goal is still processing ...")
                rate.sleep()
                if self.stop.data:
                    self.action_client.cancel_goal() #PAUSE
                    rospy.loginfo("Cancel requested, aborting ...")
                    self.stop.data = False
                    return
                    
            rospy.loginfo("Hooray! got on the right pose ...")
            
            i += 1


if __name__ == '__main__':
    rospy.init_node("RowFollower")
    try:
        node = RowFollower()
        rate = rospy.Rate(0.5)

        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
