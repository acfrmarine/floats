#!/bin/bash

source /etc/profile
source /opt/ros/kinetic/setup.bash
source /home/pi/.bashrc
source /home/pi/catkin_ws/devel/setup.bash
export ROS_IP=fp1
export ROS_MASTER_URI=http://fx1:11311

roslaunch float_control thruster_control.launch
