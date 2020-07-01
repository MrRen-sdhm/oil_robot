# This is the Oil manipulator's ROS package repository, which contains the driver of Oil. 

#### Oil is a 7-DOF manipulator developed and designed by Intelligent Robot Laboratory of HUST AIA.

###### Function list：

​	1. Communicate with robot through Ethernet and use Modbus-TCP protocol

​	2. Communicate with gripper through SerialPort and use RS485 protocol

​	3. Communicate with ROS Moveit！through ROS Topic and Action

​	4. Communicate with HMI through ROS Topic



#### Some commands to use this package:

1、Start the Oil's driver.

```
roslaunch oil_driver oil_driver.launch
```

2、Build the driver package only.

```
cd ~/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="oil_driver"
```

