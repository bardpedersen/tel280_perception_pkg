# A ROS package for person following

Simple ROS package that show cases turtlebot3 following an person in Gazebo simulation. 

clone this repository under `catkin_wd/src` an build your workspace with `catkin_make`.

Refer to following commands to install deps.

```bash
sudo apt install ros-noetic-ros-numpy
sudo apt install ros-noetic-turtlebot3*
pip3 install open3d
pip3 install transforms3d
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/$USER/catkin_ws/src/tel280_perception_pkg/models

sudo apt install ros-noetic-slam-toolbox*
rosservice call /slam_toolbox/save_map "name:                             
  data: '/home/$USER/map.pgm'"
roslaunch tel280_perception_pkg slam.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```

For slam localization, drive the robot around a litlle, set the initial pose, then deserialize the map. 

You can launch the person following node with; 

```bash
roslaunch tel280_perception_pkg person_follow.launch 
```

```bash
rostopic pub -1 /stop std_msgs/Bool "data: true"
```


![Turtlebot3 following a person](assets/output.gif)