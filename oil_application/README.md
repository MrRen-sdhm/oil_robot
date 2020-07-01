# This is the Oil manipulator's ROS package repository, which contains the application of Oil. 

#### Oil is a 7-DOF manipulator developed and designed by Intelligent Robot Laboratory of HUST AIA.

###### Function list：

​			1. Bringup the Oil manipulator.

​			2. Oil hand eye calibration by easy_handeye.

#### Some notes：

Hand eye calibration depend on some packages : [vision_visp](https://github.com/lagadic/vision_visp)、 [aruco_tracker](https://github.com/pal-robotics/aruco_ros)、[easy_handeye](https://github.com/IFL-CAMP/easy_handeye)

Calibration environment configuration please see calibration/[CalibrationTutorial.md](calibration/CalibrationTutorial.md)

#### Some commands to use this package:

1、Bringup the robot without gripper.

This command will start oil_driver、oil_hmi

```
roslaunch oil_application oil_bringup.launch
```

2、Hand eye calibration on oil.

Those command will start oil_driver、oil_hmi、realsense2_camera、aruco_tracker、easy_handeye

Start calibration program:

```
roslaunch oil_application eye_on_hand_calibration.launch
```

Show the calibration result:

```
roslaunch oil_application calib_result_show.launch
```

