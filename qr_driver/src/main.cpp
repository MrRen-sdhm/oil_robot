#include <unistd.h>
#include "ros/ros.h"
//#include "yinshi_driver/HandControl.h"

#include "qr_driver.h"


using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_driver");
    ros::NodeHandle nh;

    QRDriver qrDriver(&nh);
//    QRDriver qrDriver;
    qrDriver.SelectDir(); // 先设置方向
    sleep(0.5);

    qrDriver.GetPose(); // 循环获取当前位置

//    ros::spin();

    return 0;
}


