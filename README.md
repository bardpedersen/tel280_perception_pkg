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
```

You can launch the person following node with; 

```bash
roslaunch tel280_perception_pkg person_follow.launch 
```


![Turtlebot3 following a person](assets/output.gif)