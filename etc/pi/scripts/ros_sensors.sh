#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/pi/.bashrc
#connect-ros-tx2
export ROS_IP=172.16.154.90
export ROS_MASTER_URI=http://172.16.154.89:11311
source /home/pi/catkin_ws/devel/setup.bash
roslaunch floatpi sensors.launch
