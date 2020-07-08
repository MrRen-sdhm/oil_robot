#include <unistd.h>
#include "ros/ros.h"
//#include "yinshi_driver/HandControl.h"

#include "qr_driver.h"


using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_driver");
    ros::NodeHandle nh("~");

    string serial_name;
    nh.param<string>("serial_name", serial_name, "/dev/ttyUSB0");
    printf("[INFO] serial_name:%s\n", serial_name.c_str());

    QRDriver qrDriver(&nh, serial_name);
    qrDriver.SelectDir(); // 先设置方向
    sleep(1); // 延时1s后开始读取

    qrDriver.GetPose(); // 循环获取当前位置

//    ros::spin();

    return 0;
}


