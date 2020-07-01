# This is the Oil manipulator's ROS package repository, which contains multiple ROS functional packages. 

#### Oil is a 7-DOF manipulator developed and designed by Intelligent Robot Laboratory of HUST AIA.

This repository contains the following ROS packages of Oil manipulator：

- **oil_driver** -- Oil manipulator driver, used to control with the robot.

  It's placed in another repository: https://github.com/MrRen-sdhm/oil_driver

  ###### Function list：

  ​	1. Communicate with robot through Ethernet and use Modbus-TCP protocol

  ​	2. Communicate with gripper through SerialPort and use RS485 protocol

  ​	3. Communicate with ROS Moveit！through ROS Topic and Action

  ​	4. Communicate with HMI through ROS Topic

- **oil_hmi** -- Oil HMI, used to control the oil throug GUI.

  It is written in Python and the UI of it is written in PyQt5. It communicates with driver through ROS Topic, and use Moveit！to plan motion path.

  ###### Function list：

  ​	1. Feedback joint positions of the robot.

  ​	2. Enabling or Non-Enabling the Robot.

  ​	3. Set current position as zero position.

  ​	4. Save current joint positions to xml.

  ​	5. Emergency stop of the robot.

- **oil_description**

  It's a description of Oil manipulator, created by [sw_urdf_exporter](http://wiki.ros.org/sw_urdf_exporter)

- **oil_moveit_config**

  It's the Moveit! configuration of Oil manipulator, which created by [moveit setup assistant](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)

- **oil_application**

  It's a package which Used to implement some application functions of Oil manipulator.

  Function list：

  ​	1. Bringup the Oil manipulator.

  ​	2. Oil hand eye calibration by easy_handeye.

#### Some commands to use the Oil manipulator：

1、Bringup the robot without gripper.

```
roslaunch oil_application oil_bringup.launch
```



