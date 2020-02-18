#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/pi/.bashrc
source /home/pi/catkin_ws/devel/setup.bash
roslaunch float_control dual_thruster_single_pid.launch
