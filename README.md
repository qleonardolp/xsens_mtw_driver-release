# Xsens MTw driver using ROS-Kinetic

This project aim to develop an driver on which the Xsens MTw sensors send the data in topics published through an main node managing the
connection with the Awinda base (USB port).

### Hardware

- The Xsens Hardware is an MTw Development Kit acquired between 2014-2016

### Software

- The driver is developed upon the Xsens SDK 4.6, aside ROS Kinetic on Ubuntu 16.04 LTS

## Goals

- Read at least two accelerometers at 200 Hz and publish free_acceleration @ 200 Hz

## Usage

- All dependecies (.h, .so, .c, .cpp) are on this folder structure. The CMakelist file is already configured. Clone this repository into
catkin_ws/src and do catkin_make. 
- To run the working node:
```
rosrun xsens_mtw_driver mt_w_node
```
- The other nodes DO NOT work properly
    - xsens_mtw_node: "segmentation fault (core dumped)" on spinFor();
    - xsens_mtw_node_out: print (propably) the sensor readings straight on the node terminal (caution);

## ToDo

- [ ] Study the mastercallback, mtwcallback and the in-build ROS callbacks
- [ ] Enhance the callbacks to reduce the spinOnce() duration
- [ ] Achieve 200 Hz per sensor

## Contribute

- working in progress...