# Floats

This repository contains the software for running the floats / goldfish (D3) imaging platforms.

## Requirements

TODO

- vimba [Installation](https://www.alliedvision.com/en/products/software.html)
- ceres [Installation](http://ceres-solver.org/installation.html)
- Navio2 [Github](https://github.com/emlid/Navio2.git)

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

Install the services and enable:
```bash
cd src/floats
sudo ./install_services.sh tx2 --enable
```
