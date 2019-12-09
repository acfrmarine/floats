# D3 Launcher

This package is used to start the D3 Platform. It is designed to be run from the jetson.

## Setup

The D3 Platform requires multiple components to be connected:
- Jetson: 172.16.154.89
- PI w Navio2: 172.16.154.90
- LeftCamera: 172.16.154.174
- RightCamera: 172.16.154.173

SYSTEM DIAGRAM


### Jetson

The Jetson is used to run the cameras, imu, while also running SLAM and recording data.

Add the following lines to the ~/.bashrc file if they aren't already there:
```bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://172.16.154.89:11311
export ROS_IP=172.16.154.89
```

To allow easier communication with the pi, setup ssh keys to access the pi.
```bash
ssh-copy-id -i ~/.ssh/id_rsa.pub pi@172.16.154.90
```

The following packages should be exist in the catkin workspace of the Jetson:
- d3\_launcher (this package)
- avt\_vimba\_camera
- rtimulib\_ros
- VINS-fusion


### PI

The PI with a Navio2 shield is used to read the sensors and actuate the thrusters.

Add the following bash file, at the path ~/catkin\_ws/envs/pi\_remote\_env.sh on the pi:
```bash
#!/bin/bash
export ROS_IP=172.16.154.90
export ROS_MASTER_URI=http://172.16.154.89:11311
source /hom/pi/catkin_ws/envs/pi_remote_env.sh
exec "$@"
```
Then make the file executable:
```bash
chmod +x ~/catkin_ws/envs/pi_remote_env.sh
```

Add ssh keys to communicate with the jetson (necessary?):
```bash
ssh-copy-id -i ~/.ssh/id_rsa.pub nvidia@172.16.154.89
```

The following packages should exist in the catkin workspace of the pi:
- floatpi

## Run

roslaunch d3_launcher d3.launch


## Configure to run on startup
Using systemd to handle running the ROS scripts is useful, as the service can be started, stopped and its status queried.

### Creating the Service File
On the Jetson (or whatever machine is the ROS master), add the following file to /etc/systemd/system/ros_d3.service
```bash
[Unit]
Description=start roscore
After=remote-fs.target
After=syslog.target
[Service]
ExecStart=/usr/local/bin/ros_d3.sh
Restart=on-abort
[Install]
WantedBy=multi-user.target
```
### Startup Script
Add the startup script /usr/local/bin/ros\_d3.sh (or wherever the ExecStart command says):
```bash
#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/nvidia/.bashrc
roslaunch d3_launcher d3.launch
```

### Commands

Start
```bash
sudo systemctl start ros_d3.service
```

Stop
```bash
sudo systemctl stop ros_d3.service
```

Stop
```bash
sudo systemctl status ros_d3.service
```

Start on boot
```bash
sudo systemctl enable ros_d3.service
```

Disable start on boot
```bash
sudo systemctl disable ros_d3.service
```






