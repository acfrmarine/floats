# Floats

This repository contains the software for running the floats / goldfish (D3) imaging platforms.

## Requirements

- ros [Installation](http://wiki.ros.org/melodic/Installation)
- vimba [Installation](https://www.alliedvision.com/en/products/software.html)
- ceres [Installation](http://ceres-solver.org/installation.html)
- Navio2 [Github](https://github.com/emlid/Navio2.git)
- OpenUPS Requirements [Installation](https://github.com/mini-box/openups) (see floatpi for more detailed instructions)

## Build + Installation

Change into the catkin src directory, for example:
```bash
cd ~/catkin_ws/src/
```

Clone this repo recursively, with a --recursive to pull submodules:
```bash
git clone https://github/com/acfrmarine/floats.git --recursive
```

Configure the repository, which installs CATKIN_IGNORE files into unused repositories. The platform options are tx2,pi and common.
```bash
cd floats
./configure tx2
```

Build the catkin workspace:
```bash
cd ~/catkin_ws
catkin_make
```

Install the services and enable (enable will automatically start the services on startup) :
```bash
cd src/floats
sudo ./install_services.sh tx2 --enable
```


## Running

After the services have been installed (and optionally enabled), the following services exist:
- ros_d3: (tx2) Starts up all the core ROS nodes.
- ros_cameras: (tx2) Starts the AVT cameras.
- ros_record: (tx2) Starts the recording node, which is activated when the switch is pressed.
- ros_sensors: (pi) Starts all the sensors connected to the Navio (pressure, GPS, imu, etc.)
- ros_screen: (pi) Starts the screen on the Pi

Each of the services can be controlled using systemctl, where the following operations are available:
- start: starts the service
- stop: stops the service
- restart: restarts the service
- enable: Enables the system on startup
- disable: Disables the system on startup

For format for these commands is:
```bash
sudo systemctl OPERATION SERVICE
```
For exmample to start the cameras on the tx2:
```bash
sudo systemctl start ros_cameras
```