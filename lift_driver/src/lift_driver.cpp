//
// Created by MrRen-sdhm on 20-06-29.
//

// TODO: 单独线程中发布位置及速度

#include "lift_driver.h"

#include <cstdlib>
#include <iostream>

LiftDriver::LiftDriver(ros::NodeHandle* nh, std::string ip, int port) : nh_(*nh) {
    Init(ip, port);
}

LiftDriver::~LiftDriver() {
    m_master_->modbusDisConnect();
    delete m_master_;
}

void LiftDriver::Init(std::string ip, int port) {
    // Modbus-TCP 初始化
    m_master_ = new ModbusAdapter();
    m_master_->modbusConnectTCP(ip, port);

    lift_ctl_service = nh_.advertiseService("lift_control", &LiftDriver::LiftControl, this);
    ROS_INFO("Lift control service started!");

    lift_pose_service = nh_.advertiseService("lift_pose", &LiftDriver::LiftPose, this);
    ROS_INFO("Lift pose service started!");
}

void LiftDriver::SetBackSpeed(int32_t speed) {
    std::vector<unsigned char> _Data(4);

    // 设置回零速度，先发低位后发高位
    _Data[0] = speed >> 0;
    _Data[1] = speed >> 8;
    _Data[2] = speed >> 16;
    _Data[3] = speed >> 24;

//    for(auto i : _Data) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    int ret = _write_data(6, _Data.size() / 2, (uint16_t *) _Data.data());
    if (!ret) {printf("[INFO] Set back speed cmd send failed!\n");}
    else {printf("[INFO] Set back speed cmd send succed!\n");}
}

/**
 * 设置目标速度
 * @param speed 单位：mm/s
 */
void LiftDriver::SetSpeed(uint32_t speed) {
    aim_speed = speed;
}

/**
 * dis为上升距离，单位mm
 * aim_speed为目标速度
 */
void LiftDriver::MoveUpABS(int32_t dis) {
    std::vector<unsigned char> _Data = {0x0A, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0xF4, 0x01, 0xF4, 0x01};
//    std::vector<unsigned char> _Data = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

    // 设置位置，先发低位后发高位
    _Data[0] = dis >> 0;
    _Data[1] = dis >> 8;
    _Data[2] = dis >> 16;
    _Data[3] = dis >> 24;

    // 设置速度，先发低位后发高位
    _Data[4] = aim_speed >> 0;
    _Data[5] = aim_speed >> 8;
    _Data[6] = aim_speed >> 16;
    _Data[7] = aim_speed >> 24;

//    for(auto i : _Data) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    int ret = _write_data(0, _Data.size() / 2, (uint16_t *) _Data.data());
    if (!ret) {printf("[INFO] Move cmd send failed!\n");}
    else {printf("[INFO] Move cmd send succed!\n");}

    Move(); // 发送移动指令

    running = true;
}

void LiftDriver::Move() {
    SendCmd(0);
}

/**
 * 回零
 */
void LiftDriver::BackHome() {
    SendCmd(1); // 发回零指令
}

/**
 * 急停
 */
void LiftDriver::Stop() {
    SendCmd(3);
}

/**
 * 一直向上运动
 */
void LiftDriver::MoveUp() {
    SendCmd(4);
}

/**
 * 一直向下运动
 */
void LiftDriver::MoveDown() {
    SendCmd(5);
}

/**
 * 地址8，低8位为控制指令
 * 1：目标位置移动
 * 2：回原点
 * 3：无
 * 4：停止
 * 5：上升
 * 6：下降
 */
void LiftDriver::SendCmd(int mode) {

    std::vector<unsigned char> _Data = {(unsigned char)(0x01 << mode), 0x00};

    int ret = _write_data(8, _Data.size() / 2, (uint16_t *) _Data.data());
//    printf("[DEBUG] %d\n", ret);

    usleep(1000 * 50); // 指令发出后，延时100ms
}

/**
 * 地址9
 * 1：指定位置移动完成
 * 2：回原点完成
 * 3：第1轴使能
 * 4：第2轴使能
 */


/**
 * 地址10&11，32位
 * 第1轴，当前位置，单位为脉冲
 */

/**
 * 地址12&13，32位
 * 第2轴，当前位置，单位为脉冲
 */
void LiftDriver::GetPose() {
    usleep(1000 * 50); // 延时50ms再发送数据

    uint8_t buff[8];
    if (!m_master_->modbusReadHoldReg(1, 10, 4, (uint16_t *)buff)) {
        throw runtime_error("\033[1;31mFail to read all register from controller!\033[0m\n");
    }

    uint32_t round1 = 0, round2 = 0;
    round1 |= (int32_t) buff[3] << 24;
    round1 |= (int32_t) buff[2] << 16;
    round1 |= (int32_t) buff[1] << 8;
    round1 |= (int32_t) buff[0] << 0;

    round2 |= (int32_t) buff[7] << 24;
    round2 |= (int32_t) buff[6] << 16;
    round2 |= (int32_t) buff[5] << 8;
    round2 |= (int32_t) buff[4] << 0;

//    printf("round: %d\n", ((round1 + round2) / 2) / 4000); // 4000脉冲=1mm

    cur_pose = ((round1 + round2) / 2) / 4000; // 保存当前位置，取两轴平均 4000脉冲=1mm
}

void LiftDriver::GetState() {
    uint8_t buff[2] = {0};
    if (!m_master_->modbusReadHoldReg(1, 9, 1, (uint16_t *)buff)) {
        throw runtime_error("\033[1;31mFail to read all register from controller!\033[0m\n");
    }

    back_home_done = (buff[0] & (0x01 << 1)) != 0; // 地址9第2位被置1，回零完成

    move_done_bit = buff[0] & (0x01 << 0); // 地址9第1位被置1，绝对运动完成
}

/**
 * 通过读取速度判断是否运动完成，完成前阻塞
 * 说明：发出运动指令后，PLC控制器会先将地址9中"指定位置移动完成"标志位清0，
 * 判断此值是否为变为1即可判断电机是否运动完成
 */
void LiftDriver::MoveDoneBlocking() {
    usleep(1000 * 500); // 延时，等待PLC中将运动完成标志清0
    while(true) { // 阻塞直到电机运动完成或急停
        if(move_done_bit == 1) break; // 电机运动完成，退出循环

        if(stop_flag) break; // 急停
    }
}

void LiftDriver::BackDoneBlocking() {
    usleep(1000 * 500); // 延时，等待PLC中将回零标志清0
    while (true) {
        if(back_home_done) break;

        if(stop_flag) break; // 急停
    }
}

bool LiftDriver::LiftControl(lift_driver::LiftCtl::Request &req, lift_driver::LiftCtl::Response &res) {
    if(req.command == "Stop") {
        printf("\n[INFO] Lift emergency stop.\n");
        stop_flag = true;
        res.success = true;
    }
    else if(req.command == "Speed") { // 设置速度
        if (req.pose >= 1 && req.pose <= 50) { // 限制升降机构速度范围为1-50 mm/s
            printf("[INFO] Set speed: %.2f mm/s.\n", req.pose);
            SetSpeed(req.pose);
            res.success = true;
        } else {
            printf("[WARN] Lift support speed: 1-50 mm/s.\n");
            res.success = false;
        }
    }
    else if(req.command == "Home") {
        printf("\n[INFO] Lift back home...\n");
        back_home_flag = true;
        BackDoneBlocking();
        printf("\n[INFO] Lift back home done.\n");

        res.success = true;
    }
    // 绝对运动的pose单位为mm
    else if(req.command == "Move") {
        if (req.pose >= 0 && req.pose <= 2000) { // 限制升降机构运动范围为0-2.0米
            printf("\n[INFO] Lift MoveABS to pose: %.2f.\n", req.pose);
            move_abs_flag = true;
            move_abs_dis = req.pose; // 单位：mm

            MoveDoneBlocking(); // 阻塞直到运动完成
            running = false; // 电机停止
            printf("\n[INFO] MoveABS done.\n");

            res.success = true;
        } else {
            printf("[WARN] Lift support pose: 0-2000 mm.\n");
            res.success = false;
        }
    }
    // 相对运动的pose单位为mm
    else if(req.command == "MoveUp") {
        printf("\n[INFO] MoveUP...\n");
        move_up_flag = true; // 设置标志位

        running = false; // 电机停止
        printf("\n[INFO] MoveUP done.\n");

        res.success = true;
    }
    else if(req.command == "MoveDown") {
        printf("\n[INFO] MoveDown...\n");
        move_down_flag = true; // 设置标志位

        running = false; // 电机停止
        printf("\n[INFO] MoveDown done.\n");

        res.success = true;
    }
    else {
        printf("[WARN] Unsupported command.\n");
        res.success = false;
    }

    return true;
}

bool LiftDriver::LiftPose(lift_driver::LiftPose::Request &req, lift_driver::LiftPose::Response &res) {
    if(req.command == "Pose") { // 获取电机运行状态
        res.pose = cur_pose;
    }
    else {
        printf("Supported command: Pose\n");
    }

    return true;
}