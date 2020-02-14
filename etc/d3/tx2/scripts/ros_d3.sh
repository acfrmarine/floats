#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/nvidia/.bashrc
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=172.16.154.89
export ROS_MASTER_URI=172.16.154.89:11311
roslaunch launcher d3.launch
