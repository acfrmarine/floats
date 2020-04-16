#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/pi/.bashrc
source /home/pi/catkin_ws/devel/setup.bash
roslaunch float_control depth_control_thruster_pid.launch
