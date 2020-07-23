#! /bin/bash

source /opt/ros/kinetic/setup.bash    # 启动ros环境
source ~/catkin_ws/devel/setup.bash
# gnome-terminal -x bash -c "roscore" & # 启动roscore命令
# sleep 2
# gnome-terminal -x bash -c "rviz" &    # 启动rviz

gnome-terminal -x bash -c "roslaunch oil_application oil_bringup.launch fake:=true"  &  # 启动机器人
# sleep 2
# gnome-terminal -x bash -c "roslaunch oil_hmi oil_hmi.launch"  &  # 启动控制界面


