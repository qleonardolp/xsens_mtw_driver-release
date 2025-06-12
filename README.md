# ROS Xsens MTw Awinda Driver

ROS driver for the Xsens (rebranded as `Movella`) MTw Awinda Kit. Xsens SDK [4.6](https://www.movella.com/products/wearables/xsens-mtw-awinda).

Please consider the [xsens_mtw_driver_ros2](https://github.com/ksomml/xsens_mtw_driver_ros2) for ROS2.

## Usage

- Connect the Awinda Station USB in your computer and run: `ros2 run xsens_mtw_driver awinda_manager`

- Undock the MTW sensor and wait until the wireless connection being established:

```
[ INFO] [1565393292.619168658]: Waiting for MTW to wirelessly connect...
[ INFO] [1565393436.611962400]: EVENT: MTW Connected -> 00342322
[ INFO] [1565393436.615162761]: Number of connected MTWs: 1. Press 'y' to start measurement or 'q' to end node.

```

- Each MTw sensor will connect at once. Remember, as described on the Xsens MTw User Manual:

| Number of sensors  | Update rate (Hz) |
|-----|-----|
|  1  | 150 |
|  2  | 120 |
|  4  | 100 |
|  6  |  75 |
|  12 |  50 |
|  18 |  40 |

## Troubleshooting

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

- XSens MTI driver reference: [xsens_mti_driver page](http://wiki.ros.org/xsens_mti_driver)
