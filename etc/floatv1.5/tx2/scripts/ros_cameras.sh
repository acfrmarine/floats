#!/bin/bash
sleep 15
source /etc/profile
source /home/nvidia/.bashrc
source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=fx1
export ROS_MASTER_URI=http://fx1:11311
roslaunch launcher mono_camera_fisheye_fixed.launch
