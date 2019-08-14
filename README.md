# Xsens MTw driver using ROS-Kinetic

This project aims to develop a driver on which the Xsens MTw sensors send the data in topics published through an main node managing the
connection with the Awinda base (USB port). Development based on [Xsens MTw SDK 4.6](https://www.xsens.com/mt-software-suite-mtw-awinda/).

### Hardware

- The Xsens Hardware is an MTw Development Kit acquired between 2014-2016

### Software

- The driver is developed upon the Xsens SDK 4.6, with ROS Kinetic on Ubuntu 16.04 LTS

## Goals

- _Read at least two accelerometers and publish free_acceleration @ 200 Hz_ : is apparently impossible once the publication rate
is bounded by the `desiredUpdateRate` value set to the **Xsens Awinda Station**. The maximun value that works with two MTw is 120 Hz as suggested
on Xsens Awinda User Manual.
- _Read free_acceleration and gyroscope data_ : done at 120 Hz for both data for two sensors.

## Usage

- All dependencies (libraries, headers and sources files) are in this folder structure. The CMakelist file is already configured. Clone this repository into
catkin_ws/src and do `catkin_make`. 

    - Connect the Awinda Station USB in your computer and run the MTw Driver node: `$ rosrun xsens_mtw_driver mt_w_node`

    - Undock the MTW sensor and wait until the wireless connection being established: 

```
[ INFO] [1565393292.619168658]: Waiting for MTW to wirelessly connect...
[ INFO] [1565393436.611962400]: EVENT: MTW Connected -> 00342322
[ INFO] [1565393436.615162761]: Number of connected MTWs: 1. Press 'y' to start measurement or 'q' to end node.

```
- The **acc_based_control** is the node to calculate a desired Torque in a joint to minimize the interaction force between an exosuit and the user...

```
$ rosrun xsens_mtw_driver acc_based_control 324 323
[ INFO] [1565821614.756475631]: Subcribed on /imu_00342324, /imu_00342323
[ INFO] [1565821614.756544099]: Publishing on /desired_Torque
```
```
$ rostopic hz /desired_Torque
average rate: 120.034
        ...
```

## ToDo

- [x] Update messages type on accBasedControl node, using `sensor_msgs::Imu` from now on;
- [ ] Study the mastercallback, mtwcallback and the in-build ROS callbacks;
- [x] Develop a node that process the difference between the free accelerations of two MTw and do some math with (acc_based_control);
- [ ] Calibrate the sensors through in-build SDK methods or through statistical data analyzed from an rosbag;
- [ ] Configure the sensors to output the accelerations relative to a sensor-fixed frame;


## Contribute

- working in progress...

## Troubleshooting

If happen some problem with device access, follow the recommendations on the [xsens_mti_driver page](http://wiki.ros.org/xsens_mti_driver):

- Make sure you are in the correct group:

```
$ ls -l /dev/ttyUSB0

crw-rw---- 1 root dialout 188, 0 May  6 16:21 /dev/ttyUSB0

$ groups

"username" adm cdrom sudo dip plugdev lpadmin sambashare
```

- Add yourself to the group: 
```
$ sudo usermod -G dialout -a $USER
$ newgrp dialout
```

## License

Checking the Xsens license... (wip)

## Trivia

![Torque Demo](media/torque_demo.mp4)
