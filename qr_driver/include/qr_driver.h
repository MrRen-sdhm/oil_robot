//
// Created by MrRen-sdhm on 20-05-30.
//

#ifndef QRDRIVER_H
#define QRDRIVER_H

#include "ros/ros.h"
//#include "yinshi_driver/HandControl.h"

#include "serial_driver.h"

class QRDriver{
public:
    QRDriver();
    QRDriver(ros::NodeHandle* nh);
    ~QRDriver();

    void Init(); // 串口初始化
    void SelectDir(); // 设置方向
    void GetPose(); // 获取位置
    void Parase(); // 解析位置

private:
    unsigned char FrameCheck(std::vector<unsigned char> _Data);
//    bool HandControl(yinshi_driver::HandControl::Request  &req, yinshi_driver::HandControl::Response &res);

    ros::NodeHandle nh_;
    ros::ServiceServer hand_ctl_service;
    SerialDriver* serialdriver_;
    uint8_t rx_buf_[22]; // 接收缓冲
};


#endif //QRDRIVER_H
