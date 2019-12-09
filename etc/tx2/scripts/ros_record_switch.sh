#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/nvidia/.bashrc
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=172.16.154.89
export ROS_MASTER_URI=http://172.16.154.89:11311
sleep 15
rosrun d3_launcher record_switch.py
