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
    ros::NodeHandle nh;

    LiftDriver lift_driver(&nh);

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


    while(ros::ok()) {
        lift_driver.MoveDone(); // 检查是否运动完成
        lift_driver.GetPose(); // 获取当前位置

        usleep(1000 * 20);
        ros::spinOnce();
    }

//    ros::spin(); // FIXME

    return 0;
}


