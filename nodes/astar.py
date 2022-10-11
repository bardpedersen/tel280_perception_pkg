#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tel280_perception_pkg.helper import SimpleNavHelpers
from nav_msgs.msg import Path

# Adapted from https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py




class AStarPlanner:

    def __init__(self, occupancy_grid, rr):
        """
            Initialize grid map for a star planning
            occupancy_grid: grid [OccupancyGrid]
            rr: robot radius[m]
        """
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid = occupancy_grid
        rospy.loginfo(str(self.occupancy_grid.info))
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.motion = self.get_motion_model()
        self.calc_obstacle_map()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        # Convert World positions to grid indexes for goal and start 
        sx_grid, sy_grid = self.calc_xy_index(sx, sy)
        gx_grid, gy_grid = self.calc_xy_index(gx, gy)
        
        # Each cell is represented with a Node 
        start_node = self.Node(sx_grid, sy_grid, 0.0, -1)
        goal_node = self.Node(gx_grid, gy_grid, 0.0, -1)
        
        # We will use two sets to keep track of unexplored and explored Nodes in Occupancy Grid
        open_set, closed_set = dict(), dict()
        
        # First add start position to open set
        open_set[self.calc_grid_index(start_node)] = start_node
        init_len_openset = len(open_set)
        
        # Lets see how many iterations it will take for us to find a goal
        iter = 0
        while 1:
            
            # If start position is not added to open set, exit with failure
            if init_len_openset == 0:
                print("Open set is empty..")
                break
            
            # Get id of currently minimal cost Node, initiallt this is the start position
            # but once we add more nodes, the selection of Node with minimal cost will ensure the optimality of the 
            # resultant path
            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[o]))
            # Get Current node , we have its id (c_id)
            current = open_set[c_id]
            
            # If the current node is the goal, GHooray, we found the goal
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Found the goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break
            
            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid in 8 directions based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                # Collision check
                if not self.verify_node(node):
                    continue
                # If the Neighbour is already in explored set, do not add again            
                if n_id in closed_set:
                    continue
                
                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node
            iter += 1
        rx, ry = self.calc_final_path(goal_node, closed_set)
        
        print("Found goal in iterations: ", iter)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x)], [
                  self.calc_grid_position(goal_node.y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x))
            ry.append(self.calc_grid_position(n.y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index):
        """
        calc grid position
        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.occupancy_grid.info.resolution
        return pos

    # Convert world point x,y to grid index grid_x, grid_y
    def calc_xy_index(self, x, y):

        grid_x = (x - self.occupancy_grid.info.origin.position.x) /  self.occupancy_grid.info.resolution
        grid_y = (y - self.occupancy_grid.info.origin.position.y) /  self.occupancy_grid.info.resolution

        return int(grid_x), int(grid_y)

    # Given grid index of node, get the index
    def calc_grid_index(self, node):
        return node.y * self.occupancy_grid.info.height + node.x

    def verify_node(self, node):
        px = self.calc_grid_position(node.x)
        py = self.calc_grid_position(node.y)

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self):

        ox = []
        oy = []

        print(str(self.occupancy_grid.info))

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.occupancy_grid.info.height)]
                                    for _ in range(self.occupancy_grid.info.width)]

        index_counter = 0
        for i in self.occupancy_grid.data:
            if i == -1 or i == 100:
                # it is unknown or obstacle add it to obstacle list
                row = index_counter % self.occupancy_grid.info.width
                column = (index_counter - index_counter % self.occupancy_grid.info.width)/self.occupancy_grid.info.width

                pox = int(row) * self.occupancy_grid.info.resolution
                poy = int(column) * self.occupancy_grid.info.resolution
                ox.append(pox)
                oy.append(poy)
            index_counter += 1

        for ix in range(self.occupancy_grid.info.width):
            x = self.calc_grid_position(ix)
            for iy in range(self.occupancy_grid.info.height):
                y = self.calc_grid_position(iy)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


class Perception:

    def __init__(self):
        self.source_topic = "/map"
        self.goal_topic = "/move_base_simple/goal"
        self.map_recieved = False
        self.map = OccupancyGrid()
        rospy.Subscriber(self.source_topic, OccupancyGrid,self.source_callback)
        rospy.Subscriber(self.goal_topic, PoseStamped,self.goal_callback)
        self.path_publisher = rospy.Publisher("plan", Path, queue_size=1)
        self.nav_helper = SimpleNavHelpers()

    def source_callback(self, map: OccupancyGrid):
        if not self.map_recieved:
            self.map = map
            self.map_recieved = True
            rospy.loginfo("Recieved A map With following properties")
            # rospy.loginfo(str(self.map.info))

    def goal_callback(self, goal_pose: PoseStamped):
        rospy.loginfo("Recieved A Goal")

        # get robot pose in map frame
        robot_pose = self.nav_helper.get_curr_robot_pose(frame="map")
        sx = robot_pose.pose.position.x
        sy = robot_pose.pose.position.y
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y
        a_star = AStarPlanner(self.map, rr=0.3)
        rx, ry = a_star.planning(sx, sy, gx, gy)
        
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()   
        
        for xx, yy in zip(rx, ry):
            pose = PoseStamped()
            pose.pose.position.x = xx + self.map.info.origin.position.x
            pose.pose.position.y = yy  + self.map.info.origin.position.y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_publisher.publish(path)
 

if __name__ == '__main__':
    rospy.init_node("Perception")
    try:
        node = Perception()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
