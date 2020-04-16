#!/bin/bash

source /etc/profile
source /home/nvidia/.bashrc
source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=fx1
export ROS_MASTER_URI=http://fx1:11311


rosrun launcher always_record.py
