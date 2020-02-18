#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/pi/.bashrc
source /home/pi/catkin_ws/devel/setup.bash
roslaunch floatpi sensors.launch gps:=false trigger:=false ir_switch:=false
