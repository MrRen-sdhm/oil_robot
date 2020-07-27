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

//    string serial_name;
//    nh.param<string>("serial_name", serial_name, "/dev/ttyUSB0");
//    printf("[INFO] serial_name:%s\n", serial_name.c_str());

    LiftDriver lift_driver(&nh, "192.168.0.50", 502);
    lift_driver.SetSpeed(5); // 设置速度，仅修改速度变量
    lift_driver.SetBackSpeed(30); // 先设置回零速度，发送给PLC

#if 0
    lift_driver.BackHome(); // 回零

    while (true) { // 必须先回零才可进行下一步操作
        lift_driver.GetState();
        if(lift_driver.back_home_done) {
//            usleep(1000 * 100);
//            lift_driver.MoveUpABS(500);
            break;
        }
    }
    printf("[INFO] Back home done.\n");
#else
//    lift_driver.MoveUpABS(50);
#endif
//    lift_driver.Stop();

    /// 绝对/相对运动
//    lift_driver.MoveUpABS(10);

//    lift_driver.GetPose();

#if 1

    ros::AsyncSpinner spinner(2); // 使用异步spinner，服务函数将在单独的线程中运行，不会阻塞主线程
    spinner.start();

    /** 主循环负责与电机通信，所有收发数据均要在主循环中完成，不可在ROS服务回调函数中发送命令，否则指令可能发送失败 **/
    while(ros::ok()) {
        /** 读取数据 **/
        lift_driver.GetPose(); // 获取当前位置
//        lift_driver.GetSpeed(); // 获取当前速度
        lift_driver.GetState(); // 获取当前速度

        /** 写入数据 **/
        if(lift_driver.stop_flag) {
            lift_driver.Stop();
            lift_driver.stop_flag = false;
        }

        if(lift_driver.move_up_flag) {
            lift_driver.MoveUp();
            lift_driver.move_up_flag = false;
        }

        if(lift_driver.move_down_flag) {
            lift_driver.MoveDown();
            lift_driver.move_down_flag = false;
        }

        if(lift_driver.move_abs_flag) {
            lift_driver.MoveUpABS(lift_driver.move_abs_dis);
            lift_driver.move_abs_flag = false;
        }

        if(lift_driver.back_home_flag) {
            lift_driver.BackHome();
            lift_driver.back_home_flag = false;
        }
    }

    spinner.stop();
    ROS_INFO("Exiting lift_driver...");

#endif
    return 0;
}


