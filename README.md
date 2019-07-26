# Xsens MTw driver using ROS-Kinetic

This project aim to develop an driver on which the Xsens MTw sensors send the data in topics published through an main node managing the
connection with the Awinda base (USB port). Development based on [Xsens MTw SDK 4.6](https://www.xsens.com/mt-software-suite-mtw-awinda/).

### Hardware

- The Xsens Hardware is an MTw Development Kit acquired between 2014-2016

### Software

- The driver is developed upon the Xsens SDK 4.6, with ROS Kinetic on Ubuntu 16.04 LTS

## Goals

- Read at least two accelerometers at 200 Hz and publish free_acceleration @ 200 Hz

## Usage

- All dependencies (.h, .so, .c, .cpp) are on this folder structure. The CMakelist file is already configured. Clone this repository into
catkin_ws/src and do catkin_make. 
- To run the working node:

```
$ rosrun xsens_mtw_driver mt_w_node
```

- The other nodes DO NOT work properly
    - xsens_mtw_node: "segmentation fault (core dumped)" on spinFor();
    - xsens_mtw_node_out: print (propably) the sensor readings straight on the node terminal (caution);

## ToDo

- [x] Publisher Vector per MTw (`ros::V_Publisher`);
- [ ] Study the mastercallback, mtwcallback and the in-build ROS callbacks;
- [ ] Enhance the callbacks to reduce the spinOnce() duration;
- [ ] Achieve 200 Hz per sensor;

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

Checking the Xsens license...