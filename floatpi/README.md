# FloatPI

This package is designed to be run on the Raspberry Pi with Navio2 Shield on the D3 and Imaging Float platforms.
It reads data from an array of sensors (gps, pressure, imu, leak) and controls the thrusters.

## Requirements
- ROS
- Navio2 Library
- spidev


### Installing Navio2 Library

Python:
```bash
git clone https://github.com/emlid/Navio2.git
cd Navio2/Python
pip install .
```

## Launch

To launch the floatpi library to only collect data from the sensors:
```bash
roslaunch floatpi sensors.launch
```

To launch the floatpi library to collect data from the sensors and control the thrusters:
```bash
roslaunch floatpi sensors_thrusters.launch
```


## Configuring OpenUPS

Follow the instructions at https://github.com/mini-box/openups , with the following additions:
- Step4: Add to /etc/nut/ups.conf instead of /etc/ups.conf
- Step5: Change mode to standalone in /etc/nut/nut.conf
- Step5: After doing the ammended steps, do 'sudo service nut start'

TODO. Startup service configuring UPS.