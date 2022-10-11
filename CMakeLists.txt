cmake_minimum_required(VERSION 3.0.2)

project(tel280_perception_pkg)

find_package(catkin REQUIRED COMPONENTS 
roscpp
rospy
std_msgs 
geometry_msgs)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs)


catkin_install_python(PROGRAMS
                    nodes/person_follower.py
                    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                    )
catkin_install_python(PROGRAMS
                    nodes/sensor_fuser.py
                    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                    )
catkin_install_python(PROGRAMS
                    nodes/noisy_odometry.py
                    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                    )                    
catkin_install_python(PROGRAMS
                    nodes/astar.py
                    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                    )     

install(DIRECTORY launch worlds urdf models rviz config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
