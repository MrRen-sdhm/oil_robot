//
// Created by MrRen-sdhm on 20-06-29.
//

#ifndef QRDRIVER_H
#define QRDRIVER_H

#include "ros/ros.h"
#include "lift_driver/LiftCtl.h"
#include "lift_driver/LiftStat.h"
#include "lift_driver/LiftPose.h"

#include "serial_driver.h"
#include "SerialPort.h"

#include <vector>

using namespace LibSerial;

class LiftDriver{
public:
    LiftDriver();
    LiftDriver(ros::NodeHandle* nh);
    ~LiftDriver();

    void Init(); // 串口初始化
    void MoveUp(int32_t round); // 向上相对运动
    void MoveDown(int32_t round); // 向下相对运动
    void MoveUpABS(int32_t round); // 向上绝对运动
    void MoveDownABS(int32_t round); // 向下绝对运动

    void MoveABS(float pose); // 绝对位置运动
    bool MoveDone(); // 阻塞到移动完成
    void BackHome(); // 回原点
    void Stop(); // 急停
    int32_t GetPose(); // 获取当前位置
    int16_t GetSpeed(); // 获取当前速度
    void SetSpeed(uint32_t speed_); // 设置目标速度
    void GetState(); // 获取当前报警状态

    bool running = false; // 电机工作标志
    bool stop_flag = false; // 急停标志
    int16_t aim_speed = 200; // 电机转速，单位：转/分钟
    int32_t cur_pose = 0; // 当前位置，单位：脉冲

private:
    unsigned short ModbusCRC16(std::vector<unsigned char> &buff, uint16_t len); // ModbusCRC16校验
    bool LiftControl(lift_driver::LiftCtl::Request  &req, lift_driver::LiftCtl::Response &res);
    bool LiftStatus(lift_driver::LiftStat::Request  &req, lift_driver::LiftStat::Response &res);
    bool LiftPose(lift_driver::LiftPose::Request  &req, lift_driver::LiftPose::Response &res);

    ros::NodeHandle nh_;
    ros::ServiceServer lift_ctl_service;
    ros::ServiceServer lift_stat_service;
    ros::ServiceServer lift_pose_service;

    SerialDriver* serialdriver_;
    unsigned char rx_buf_[20]; // 接收缓冲

    SerialPort* serial_port;
    DataBuffer read_buffer ; // 接收缓冲区
    size_t ms_timeout = 250 ; // 接收超时，即获取超时时间内的数据
};


#endif //QRDRIVER_H
