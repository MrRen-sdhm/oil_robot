//
// Created by MrRen-sdhm on 20-06-29.
//

#include <unistd.h>
#include "ros/ros.h"
//#include "yinshi_driver/HandControl.h"

#include "lift_driver.h"


using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_driver");
    ros::NodeHandle nh;
//
//    LiftDriver qrDriver(&nh);
////    LiftDriver qrDriver;
//    qrDriver.MoveUp(100000); // 先设置方向
//    sleep(0.5);
//
//    qrDriver.MoveDown(); // 循环获取当前位置
//

    LiftDriver lift_driver(&nh);
//    lift_driver.MoveDown(100000);
//    lift_driver.MoveUp(100000);
//    lift_driver.MoveDownABS(100000);
//    lift_driver.MoveUpABS(100 *4000);
//    lift_driver.MoveUpABS(1000 *4000); // 走到1m位置 1mm-4000脉冲  丝杠1转50mm，电机减速比是20

//    lift_driver.MoveABS(500); // 绝对定位0.5m
//    lift_driver.BackHome();
//    lift_driver.Stop();
    lift_driver.GetPose();
//    lift_driver.GetSpeed();
//    lift_driver.GetState();


//    while(true) {
//        sleep(1);
//        int16_t speed = lift_driver.GetSpeed();
////        printf("Speed:%d\n", speed);
//    }

//    ros::spin(); // FIXME

    return 0;
}


