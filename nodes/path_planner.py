#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tel280_perception_pkg.helper import SimpleNavHelpers
from tel280_perception_pkg.astar_planner import AStarPlanner
from nav_msgs.msg import Path

class Planning:

    def __init__(self):
        self.map_topic = "/map"
        self.goal_topic = "/move_base_simple/goal" # Take a look to RVIZ, 2D Nav Goal
        self.map_recieved = False
        self.map = OccupancyGrid() 
        rospy.Subscriber(self.map_topic, OccupancyGrid,self.map_callback)
        rospy.Subscriber(self.goal_topic, PoseStamped,self.goal_callback)
        self.path_publisher = rospy.Publisher("plan", Path, queue_size=1)
        self.nav_helper = SimpleNavHelpers()
        
        # Create an instance of AStar
        self.a_star = AStarPlanner(self.map, rr=0.3)

    def map_callback(self, map: OccupancyGrid):
        # Recieve map only once
        if not self.map_recieved:
            self.map = map
            self.map_recieved = True
            rospy.loginfo("Recieved A map With following properties")
            rospy.loginfo(str(self.map.info))

    def goal_callback(self, goal_pose: PoseStamped):
        rospy.loginfo("Recieved A Goal")

        # get robot pose in map frame
        robot_pose = self.nav_helper.get_curr_robot_pose(frame="map")
        
        # Start position is current position 
        sx = robot_pose.pose.position.x
        sy = robot_pose.pose.position.y
        
        # Goal is coming from RVIZ  2D Nav Goal Publisher
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y
        
        # Plan from curr position to Goal position 
        
        try:
            rx, ry = self.a_star.planning(sx, sy, gx, gy)
        except:
            print("Failed to find a path, Are you sure that the Goal is not too close to obstacles ?")
            print("Try with a lower robot radius")
            return
        
        # Publish the plan to RVIZ
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()   
        
        # Th plan is in Grid Cell coordinates, convert it to "/map" frame 
        for xx, yy in zip(rx, ry):
            pose = PoseStamped()
            pose.pose.position.x = xx + self.map.info.origin.position.x
            pose.pose.position.y = yy  + self.map.info.origin.position.y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_publisher.publish(path)
 

if __name__ == '__main__':
    rospy.init_node("Planning")
    try:
        node = Planning()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
