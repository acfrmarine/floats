#!/bin/bash
sleep 15
#su -s "/bin/bash" -c "/home/nvidia/ros_cameras_user.sh"  nvidia
source /etc/profile
source /home/nvidia/.bashrc
source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=172.16.154.89
export ROS_MASTER_URI=http://172.16.154.89:11311
roslaunch launcher stereo_camera_two_nodes_xtrig.launch
