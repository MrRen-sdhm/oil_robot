//
// Created by MrRen-sdhm on 20-06-29.
//

#ifndef LIFT_DRIVER_H
#define LIFT_DRIVER_H

#include "ros/ros.h"
#include "lift_driver/LiftCtl.h"
#include "lift_driver/LiftStat.h"
#include "lift_driver/LiftPose.h"

#include "modbusadapter.h"

#include <vector>

using namespace std;

class LiftDriver{
public:
    LiftDriver(ros::NodeHandle* nh, std::string ip, int port);
    ~LiftDriver();

    void Init(string ip, int port); // 串口初始化
    void MoveUp(); // 向上持续运动
    void MoveDown(); // 向下持续运动
    void MoveUpABS(int32_t dis); // 向上绝对运动
    void Move();

    void SetBackSpeed(int32_t speed);
    void SetSpeed(uint32_t speed_); // 设置目标速度

    void MoveDoneBlocking(); // 阻塞到移动完成
    void BackDoneBlocking(); // 阻塞到回零完成
    void BackHome(); // 回原点
    void Stop(); // 急停

    void GetPose(); // 获取当前位置
    void GetState(); // 获取当前报警状态

    void SendCmd(int mode);

    bool running = false; // 电机工作标志
    bool stop_flag = false; // 急停标志
    int16_t aim_speed = 5; // 电机转速，单位：mm/s

    // 运动控制标志位，目的在于将发送指令的过程放到主循环中
    bool move_up_flag = false; // 向上相对运动标志
    bool move_down_flag = false; // 向下相对运动标志
    bool move_abs_flag = false; // 绝对运动标志
    float move_abs_dis = 0; // 绝对运动的距离，单位mm
    bool back_home_flag = false; // 回零标志位

    bool back_home_done = false;
    uint8_t move_done_bit = 0;

    int32_t cur_pose = 0; // 当前位置，单位：脉冲
    int16_t cur_speed = 0; // 当前速度，单位：转/分钟

private:
    bool LiftControl(lift_driver::LiftCtl::Request  &req, lift_driver::LiftCtl::Response &res);
    bool LiftPose(lift_driver::LiftPose::Request  &req, lift_driver::LiftPose::Response &res);

    ros::NodeHandle nh_;
    ros::ServiceServer lift_ctl_service;
    ros::ServiceServer lift_stat_service;
    ros::ServiceServer lift_pose_service;

    /// modbus相关参数
    ModbusAdapter *m_master_;        // modbus服务器
    // modbus读保持寄存器数据, 使用uint16_t类型写入虚拟寄存器, 使用数据时将虚拟寄存器数据转换为int16或int32类型即可
    int _read_data(int addr, int len, uint16_t* data) {
        int rect = m_master_->modbusReadHoldReg(1, addr, len, data);
        return rect;
    }

    // modbus写保持寄存器数据, 使用uint16_t类型写入, int16或int32类型数据需转换为uint16类型
    int _write_data(int addr, int len, const uint16_t* data) {
        return m_master_->modbusWriteData(1, MODBUS_FC_WRITE_MULTIPLE_REGISTERS, addr, len, data);
    }
};


#endif //LIFT_DRIVER_H
