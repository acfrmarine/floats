#!/bin/bash

source /etc/profile
source /opt/ros/kinetic/setup.bash
source /home/pi/.bashrc
source /home/pi/catkin_ws/devel/setup.bash
export ROS_IP=172.16.154.92
export ROS_MASTER_URI=http://172.16.154.93:11311

roslaunch floatpi sensors.launch gps:=true trigger:=false ir_switch:=false
