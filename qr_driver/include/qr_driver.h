//
// Created by MrRen-sdhm on 20-05-30.
//

#ifndef QRDRIVER_H
#define QRDRIVER_H

#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
//#include "yinshi_driver/HandControl.h"

#include "SerialPort.h"

using namespace LibSerial;
using namespace std;

class QRDriver{
public:
    QRDriver(ros::NodeHandle* nh, string serial_Name);
    ~QRDriver();

    void Init(string serial_Name); // 串口初始化
    void SelectDir(); // 设置方向
    void GetPose(); // 获取位置
    void Parsing(); // 解析位置

private:
//    bool HandControl(yinshi_driver::HandControl::Request  &req, yinshi_driver::HandControl::Response &res);

    ros::NodeHandle nh_;
    ros::Publisher pub = nh_.advertise<std_msgs::Int16MultiArray>("qr_pose", 10);
    ros::ServiceServer hand_ctl_service;

    SerialPort* serial_port;
    DataBuffer read_buffer ; // 接收缓冲区
    size_t ms_timeout = 250 ; // 接收超时，即获取超时时间内的数据
};


#endif //QRDRIVER_H
