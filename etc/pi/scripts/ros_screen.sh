#!/bin/bash
#source /opt/ros/kinetic/setup.bash
#source /home/pi/.bashrc
#export ROS_IP=172.16.154.90
#export ROS_MASTER_URI=http://172.16.154.89:11311
#source /home/pi/catkin_ws/devel/setup.bash
#export DISPLAY=:0
#rosrun floatpi diver-screen-ros.py
echo "Running the script"
sudo su -s "/bin/bash" -c "/home/pi/ros_screen_user.sh" pi
