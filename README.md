# This is the Oil manipulator's ROS package repository, which contains multiple ROS functional packages. 

#### Oil is a 7-DOF manipulator developed and designed by Intelligent Robot Laboratory of HUST AIA.

This repository contains the following ROS packages of Oil manipulator：

- **oil_driver** -- Oil manipulator driver, used to control with the robot.

  ###### Function list：

  ​	1. Communicate with robot through Ethernet and use Modbus-TCP protocol

  ​	2. Communicate with lift through Ethernet and use Modbus-TCP protocol

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

1、Bringup the robot.

```bash
# real robot
roslaunch oil_application oil_bringup.launch
# fake robot
roslaunch oil_application oil_bringup.launch fake:=true
```

2、Bringup the robot and camera.

```bash
# real robot
roslaunch oil_application oil_with_realsense.launch
# fake robot
roslaunch oil_application oil_with_realsense.launch fake:=true
```

3、Control the lift only.

```bash
roslaunch lift_driver lift_driver.launch
# Open another terminal
# Back home
rosrun lift_driver lift_control_client.py "Home"
# Move abs, unit is mm
rosrun lift_driver lift_control_client.py "Move" 10
# Stop
rosrun lift_driver lift_control_client.py "Stop"
```

4、Hand eye calibration.

1. Start Calibration

    ```bash
    # real robot
    roslaunch oil_application eye_on_hand_calibration.launch
    # fake robot
    roslaunch oil_application eye_on_hand_calibration.launch fake:=true
    ```

2. Save the calibration result

   copy the transform matrix to publish_static_tf.launch

3. Verify calibration

   ```bash
   # real robot
   roslaunch oil_application calib_verification.launch
# fake robot
   roslaunch oil_application calib_verification.launch fake:=true
   
   # robot move reference to aruco_marker
   rosrun oil_application calib_verify.py
   ```
   
   