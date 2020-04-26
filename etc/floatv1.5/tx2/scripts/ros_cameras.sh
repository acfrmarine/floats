#!/bin/bash
sleep 15
source /etc/profile
source /home/nvidia/.bashrc
source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=172.16.154.93
export ROS_MASTER_URI=http://172.16.154.93:11311
roslaunch launcher mono_camera_fisheye_fixed.launch
