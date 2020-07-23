//
// Created by MrRen-sdhm on 20-06-29.
//

#include <unistd.h>
#include "ros/ros.h"
#include "lift_driver.h"


using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lift_driver");
    ros::NodeHandle nh("~");

    string serial_name;
    nh.param<string>("serial_name", serial_name, "/dev/ttyUSB0");
    printf("[INFO] serial_name:%s\n", serial_name.c_str());

    LiftDriver lift_driver(&nh, serial_name);
    lift_driver.SetSpeed(10); // 10mm/s

/// 绝对/相对运动
//    lift_driver.MoveDown(100000);
//    lift_driver.MoveUp(100000);

//    lift_driver.MoveDownABS(100000);
//    lift_driver.MoveUpABS(100000);
//    lift_driver.MoveUpABS(0);
//    lift_driver.MoveUpABS(1000 *4000); // 走到1m位置 1mm-4000脉冲  丝杠1转50mm，电机减速比是20

//    lift_driver.MoveABS(500); // 绝对定位0.5m

//    lift_driver.BackHome();
//    lift_driver.Stop();
//    lift_driver.GetPose(); // abs=0时 round=-359
//    lift_driver.GetSpeed();
//    lift_driver.GetState();

    ros::AsyncSpinner spinner(2); // 使用异步spinner，服务函数将在单独的线程中运行，不会阻塞主线程
    spinner.start();

    /** 主循环负责与电机通信，所有收发数据均要在主循环中完成，不可在ROS服务回调函数中发送命令，否则指令可能发送失败 **/
    while(ros::ok()) {
        /** 读取数据 **/
        lift_driver.GetPose(); // 获取当前位置
        lift_driver.GetSpeed(); // 获取当前速度

        /** 写入数据 **/
        if(lift_driver.stop_flag) {
            lift_driver.Stop();
            lift_driver.stop_flag = false;
        }

        if(lift_driver.move_up_flag) {
            lift_driver.MoveUp(lift_driver.move_up_round);
            lift_driver.move_up_flag = false;
        }

        if(lift_driver.move_down_flag) {
            lift_driver.MoveDown(lift_driver.move_down_round);
            lift_driver.move_down_flag = false;
        }

        if(lift_driver.move_abs_flag) {
            lift_driver.MoveUpABS(lift_driver.move_abs_round);
            lift_driver.move_abs_flag = false;
        }

        if(lift_driver.back_home_flag) {
            lift_driver.BackHome();
            lift_driver.back_home_flag = false;
        }
    }

    spinner.stop();
    ROS_INFO("Exiting lift_driver...");

    return 0;
}


