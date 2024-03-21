# Xsens MTw driver for ROS2 (Foxy/Humble)
Migrate this project to ROS2.
---
This project contains a driver on which the Xsens MTw sensors send the data in topics published through a node managing the
connection with the Awinda base (USB port).

### Hardware

- The Xsens Hardware is an MTw Development Kit acquired between 2014-2016

### Software

- The driver is developed upon the Xsens SDK 4.6, with ROS2 on Ubuntu 22.04 LTS

## Usage

- All dependencies (libraries, headers and sources files) are in this folder structure. The _CMakeList.txt_ file is ready to use, contemplating the mininum XSens SDK required. Clone this repository into your ros2_ws/src and do `colcon build` in ros2_ws/. 

- Connect the Awinda Station USB in your computer and run the MTw Driver node: `$ ros2 run xsens_mtw_driver mt_w_manager`

- Undock the MTW sensor and wait until the wireless connection being established: 

```
[ INFO] [1565393292.619168658]: Waiting for MTW to wirelessly connect...
[ INFO] [1565393436.611962400]: EVENT: MTW Connected -> 00342322
[ INFO] [1565393436.615162761]: Number of connected MTWs: 1. Press 'y' to start measurement or 'q' to end node.

```

- Each MTw sensor will connect at once. Remember, as described on the Xsens MTw User Manual:

| MTw  | desiredUpdateRate (max) |
|------|-------------------------|
|  1   |           150 Hz        |
|  2   |           120 Hz        |
|  4   |           100 Hz        |
|  6   |            75 Hz        |
|  12  |            50 Hz        |
|  18  |            40 Hz        |

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

- BSD license
