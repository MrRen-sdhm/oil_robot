//
// Created by MrRen-sdhm on 20-06-29.
//

// TODO: 单独线程中发布位置及速度

#include "lift_driver.h"

#include <cstdlib>
#include <iostream>

LiftDriver::LiftDriver(ros::NodeHandle* nh, std::string serialDeviceName) : nh_(*nh) {
    Init(serialDeviceName);
}

LiftDriver::~LiftDriver() {
    serial_port->Close();
    delete serial_port;
}

void LiftDriver::Init(std::string serial_name) {
    try {
        serial_port = new SerialPort(serial_name, BaudRate::BAUD_9600, CharacterSize::CHAR_SIZE_8,
                                      FlowControl::FLOW_CONTROL_NONE, Parity::PARITY_EVEN, StopBits::STOP_BITS_1);
    }
    catch  (const OpenFailed&) {
        throw std::runtime_error("Open serial of lift failed!\n");
    }

    lift_ctl_service = nh_.advertiseService("lift_control", &LiftDriver::LiftControl, this);
    ROS_INFO("Lift control service started!");

    lift_stat_service = nh_.advertiseService("lift_status", &LiftDriver::LiftStatus, this);
    ROS_INFO("Lift status service started!");
    lift_pose_service = nh_.advertiseService("lift_pose", &LiftDriver::LiftPose, this);
    ROS_INFO("Lift pose service started!");
}

/**
 * 向上运动， 相对运动（PR1），单位：转
 * 设置指令: 01 10 62 08 00 06 0C 00 41 FF FE 79 60 00 C8 00 32 00 32
 * 00 41 表示相对运动 见PR9.00 / FF FE 79 60 代表-100000转  FF FE为高位 79 60为低位 见PR9.01（高位）及PR9.02（低位）
 * 00 C8 代表速度 / 00 32 代表加速度 / 00 32 代表减速度
 */
void LiftDriver::MoveUp(int32_t round) {
    if(running) return; // 电机正在运行
    running = true; // 电机启动

    usleep(1000 * 50); // 延时
    round = -round; // 向上时脉冲数为负
    std::vector<unsigned char> _Data = {0x01, 0x10, 0x62, 0x08, 0x00, 0x06, 0x0C, 0x00, 0x41};
    // 脉冲数
    _Data.push_back(round >> 24);
    _Data.push_back(round >> 16);
    _Data.push_back(round >> 8 );
    _Data.push_back(round >> 0 );

    // 速度/加速度/减速度
    _Data.push_back(aim_speed >> 8);
    _Data.push_back(aim_speed >> 0);
    _Data.push_back(0x00);
    _Data.push_back(0x32);
    _Data.push_back(0x00);
    _Data.push_back(0x32);

    // CRC校验
    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

//    for(auto i : _Data) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    serial_port->Write(_Data);

    usleep(1000 * 50); // 等待50ms

    // PR1运行
    std::vector<unsigned char> _Data1 = {0x01, 0x06, 0x60, 0x02, 0x00, 0x11};
    ret = ModbusCRC16(_Data1, _Data1.size());
    _Data1.push_back(ret >> 8);
    _Data1.push_back(ret >> 0);

//    printf("\n");
//    for(auto i : _Data1) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    serial_port->Write(_Data1);

//    MoveDone(); // 阻塞到移动完成
//    running = true;
    usleep(1000 * 50);

//    sleep(10);
//    while(cur_speed != 0) {
//        printf("[INFO] speed:%d\n", cur_speed);
//    }
}

/**
 * 向下运动，相对运动（PR0），单位：转
 * 设置指令: 01 10 62 00 00 06 0C 00 41 00 01 86 A0 00 C8 00 32 00 32
 * 00 41 表示相对运动 见PR9.00 / 00 01 86 A0 代表100000转  00 01为高位 86 A0为低位 见PR9.01（高位）及PR9.02（低位）
 * 00 C8 代表速度 / 00 32 代表加速度 / 00 32 代表减速度
 * 运动指令： 01 06 60 02 00 10
 */
void LiftDriver::MoveDown(int32_t round) {
    if(running) return; // 电机正在运行
    running = true; // 电机启动

    usleep(1000 * 50); // 延时
    std::vector<unsigned char> _Data = {0x01, 0x10, 0x62, 0x00, 0x00, 0x06, 0x0C, 0x00, 0x41};
    // 脉冲数
    _Data.push_back(round >> 24);
    _Data.push_back(round >> 16);
    _Data.push_back(round >> 8 );
    _Data.push_back(round >> 0 );

    // 速度/加速度/减速度
    _Data.push_back(aim_speed >> 8);
    _Data.push_back(aim_speed >> 0);
    _Data.push_back(0x00);
    _Data.push_back(0x32);
    _Data.push_back(0x00);
    _Data.push_back(0x32);

    // CRC校验
    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

//    for(auto i : _Data) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    serial_port->Write(_Data);

    usleep(1000 * 50); // 等待50ms

    // PR0运行
    std::vector<unsigned char> _Data1 = {0x01, 0x06, 0x60, 0x02, 0x00, 0x10};
    ret = ModbusCRC16(_Data1, _Data1.size());
    _Data1.push_back(ret >> 8);
    _Data1.push_back(ret >> 0);

//    printf("\n");
//    for(auto i : _Data1) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    serial_port->Write(_Data1);

//    MoveDone(); // 阻塞到移动完成
//    running = true;
    usleep(1000 * 50);
}

/**
 * 向上运动，绝对运动（PR0）
 * round为脉冲数
 */
void LiftDriver::MoveUpABS(int32_t round) {
    usleep(1000 * 10); // 延时
    round = -round; // 向上时脉冲数为负
    std::vector<unsigned char> _Data = {0x01, 0x10, 0x62, 0x18, 0x00, 0x06, 0x0C, 0x00, 0x01}; // 第四个字节为0x18 最后一个字节为0x01
    // 脉冲数
    _Data.push_back(round >> 24);
    _Data.push_back(round >> 16);
    _Data.push_back(round >> 8 );
    _Data.push_back(round >> 0 );

    // 速度/加速度/减速度
    _Data.push_back(aim_speed >> 8);
    _Data.push_back(aim_speed >> 0);
    _Data.push_back(0x00);
    _Data.push_back(0x32);
    _Data.push_back(0x00);
    _Data.push_back(0x32);

    // CRC校验
    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

//    for(auto i : _Data) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    serial_port->Write(_Data);

    usleep(50000 * 10); // 等待50ms

    // PR1运行
    std::vector<unsigned char> _Data1 = {0x01, 0x06, 0x60, 0x02, 0x00, 0x13};
    ret = ModbusCRC16(_Data1, _Data1.size());
    _Data1.push_back(ret >> 8);
    _Data1.push_back(ret >> 0);

//    printf("\n");
//    for(auto i : _Data1) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    serial_port->Write(_Data1);

//    MoveDone(); // 阻塞到移动完成
    running = true;
    usleep(1000 * 100); // 运动指令发出后，延时100ms才可读取速度，等待电机运动
}

/**
 * 绝对位置运动，单位：mm
 * @param pose： 目标位置
 */
void LiftDriver::MoveABS(float pose) {
    int32_t round = pose *4000; // 1mm-4000脉冲 —— 丝杠1转50mm，电机减速比是20
    MoveUpABS(round);
}

/**
 * 向下运动，绝对运动（PR1），单位：转
 */
void LiftDriver::MoveDownABS(int32_t round) {
    std::vector<unsigned char> _Data = {0x01, 0x10, 0x62, 0x10, 0x00, 0x06, 0x0C, 0x00, 0x01}; // 第四个字节为0x10 最后一个字节为0x01
    // 脉冲数
    _Data.push_back(round >> 24);
    _Data.push_back(round >> 16);
    _Data.push_back(round >> 8 );
    _Data.push_back(round >> 0 );

    // 速度/加速度/减速度
    _Data.push_back(aim_speed >> 8);
    _Data.push_back(aim_speed >> 0);
    _Data.push_back(0x00);
    _Data.push_back(0x32);
    _Data.push_back(0x00);
    _Data.push_back(0x32);

    // CRC校验
    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

//    for(auto i : _Data) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    serial_port->Write(_Data);

    usleep(50000 * 10); // 等待50ms

    // PR0运行
    std::vector<unsigned char> _Data1 = {0x01, 0x06, 0x60, 0x02, 0x00, 0x12};
    ret = ModbusCRC16(_Data1, _Data1.size());
    _Data1.push_back(ret >> 8);
    _Data1.push_back(ret >> 0);

//    printf("\n");
//    for(auto i : _Data1) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    serial_port->Write(_Data1);

//    MoveDone(); // 阻塞到移动完成
    running = true;
    usleep(1000 * 100); // 运动指令发出后，延时100ms才可读取速度，等待电机运动
}

void LiftDriver::BackHome() {

    usleep(1000 * 50); // 等待50ms
    // 先切换到回零模式
    std::vector<unsigned char> _Data = {0x01, 0x06, 0x60, 0x0A, 0x00, 0x01};
    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    serial_port->Write(_Data);

    usleep(1000 * 50); // 等待50ms

    // 设置回零速度
    std::vector<unsigned char> _Data1 = {0x01, 0x06, 0x60, 0x0F, 0x01, 0x2C};
    ret = ModbusCRC16(_Data1, _Data1.size());
    _Data1.push_back(ret >> 8);
    _Data1.push_back(ret >> 0);

    serial_port->Write(_Data1);

    usleep(1000 * 50); // 等待50ms

    // 回零
    std::vector<unsigned char> _Data2 = {0x01, 0x06, 0x60, 0x02, 0x00, 0x20};
    ret = ModbusCRC16(_Data2, _Data2.size());
    _Data2.push_back(ret >> 8);
    _Data2.push_back(ret >> 0);

    serial_port->Write(_Data2);

    MoveDone(); // 阻塞到移动完成
}

void LiftDriver::Stop() {
    usleep(1000 * 50);
    std::vector<unsigned char> _Data = {0x01, 0x06, 0x60, 0x02, 0x00, 0x40};

    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    serial_port->Write(_Data);
}


int32_t LiftDriver::GetPose() {
    usleep(1000 * 50); // 延时50ms再发送数据
    std::vector<unsigned char> _Data = {0x01, 0x03, 0x60, 0x2C, 0x00, 0x02};

    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    serial_port->Write(_Data);

//    printf("\nTx:");
//    for(auto i : _Data) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    usleep(1000 * 50); // 延时50ms再读取数据

    int32_t round = 0; // 电机当前位置（脉冲数）

    try {
        serial_port->Read(read_buffer, 0, ms_timeout) ; // 0表示在超时时间内尽可能多的读取数据
    }
    catch (const ReadTimeout&) { // 超时即触发异常，注意此处的超时并不是错误
        int len = read_buffer.size();
//        printf("len: %d\n", len);
        if (len == 9) { // 正确接收到9个响应字节
//            printf("Rx:");
//            for (int i = 0; i < len; i++)
//                printf("%.2X ", read_buffer[i]);
//            puts("");

            round |= (int32_t) read_buffer[3] << 24;
            round |= (int32_t) read_buffer[4] << 16;
            round |= (int32_t) read_buffer[5] << 8;
            round |= (int32_t) read_buffer[6] << 0;
//            int32_t round = ((int32_t)rx_buf_[3] << 24) | ((int32_t)rx_buf_[4] << 16) | ((int32_t)rx_buf_[5] << 8) | (int32_t)rx_buf_[6];
        }
    }
//    printf("round: %d\n", round);

    cur_pose = round; // 保存当前位置

//    unsigned short ret1 = ModbusCRC16(read_buffer, 7);
//    printf("crc: %.2X\n", ret1);

    return round;
}

/**
 * 获取速度，单位：转/分钟
 * @return
 */
int16_t LiftDriver::GetSpeed() {
    usleep(1000 * 50); // 等待10ms
    std::vector<unsigned char> _Data = {0x01, 0x03, 0x0B, 0x09, 0x00, 0x01};

    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    serial_port->Write(_Data);

//    printf("\n");
//    for(auto i : _Data) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    // 延时读取数据
    usleep(1000 * 50); // 等待50ms

    int16_t speed = 0;

    try {
        serial_port->Read(read_buffer, 0, ms_timeout) ; // 0表示在超时时间内尽可能多的读取数据
    }
    catch (const ReadTimeout&) { // 超时即触发异常，注意此处的超时并不是错误
        int len = read_buffer.size();
//        printf("len: %d\n", len);
        if (len == 7) { // 正确接收到9个响应字节
//            printf("Rx:");
//            for (int i = 0; i < len; i++)
//                printf("%.2X ", read_buffer[i]);
//            puts("");

            speed |= (int32_t)read_buffer[3] << 8;
            speed |= (int32_t)read_buffer[4] << 0;

            cur_speed = speed; // 保存当前速度
//            printf("Speed: %d\n", speed);
            return speed;
        }
    }

    return -1; // 未读取到速度
}

/**
 * 设置目标速度 1mm/s —— 24转/分钟
 * @param speed_ 单位：mm/s
 */
void LiftDriver::SetSpeed(uint32_t speed) {
    aim_speed = speed * 24;
}

void LiftDriver::GetState() {
    std::vector<unsigned char> _Data = {0x01, 0x03, 0x0B, 0x05, 0x00, 0x01};

    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    serial_port->Write(_Data);

//    printf("\nTx:");
//    for(auto i : _Data) {
//        printf("%.2X ", i);
//    }
//    printf("\n");

    usleep(1000 * 50); // 延时50ms再读取数据

    int32_t round = 0; // 电机当前位置（脉冲数）

    try {
        serial_port->Read(read_buffer, 0, ms_timeout) ; // 0表示在超时时间内尽可能多的读取数据
    }
    catch (const ReadTimeout&) { // 超时即触发异常，注意此处的超时并不是错误
        int len = read_buffer.size();
//        printf("len: %d\n", len);
        if (len == 9) { // 正确接收到9个响应字节
//            printf("Rx:");
//            for (int i = 0; i < len; i++)
//                printf("%.2X ", read_buffer[i]);
//            puts("");
        }
    }
}

/**
 * 通过读取速度判断是否运动完成，完成前阻塞
 */
void LiftDriver::MoveDoneBlocking() {
    bool move_flag = false;
    int cnt = 0; // FIXME 调试用
    uint64_t wait_cnt = 0; // 等待电机运动
    while(true) { // 阻塞直到电机运动完成或急停
        cnt++;

        if(!move_flag && cur_speed != 0) { // 未运动->运动 NOTE：若电机未实际转动，无法进入此判断！！
            printf("[DEBUG] start move...\n");
            move_flag = true;
        }
        else if(!move_flag && cur_speed == 0) { // 电机未转动，正常情况下应该在一定延时后运动，若未运动，说明发送运动指令失败
            // 若长时间无法从为运动->运动，那么跳出循环
            usleep(1000 * 10);
            wait_cnt++;
//            printf("[DEBUG] wait cnt:%d\n", wait_cnt);
            if(wait_cnt > 150) { // 约等待1s，还未运行
                printf("[INFO] Motor has not been running!\n");
                break;
            }
        }
        else if(move_flag && cur_speed == 0) { // 运动->停止
            printf("[DEBUG] move done.\n");
            move_flag = false;
            break; // 跳出循环，退出服务函数
        }

        if(stop_flag) { // 急停
            printf("[DEBUG] emergency stop.\n");
//            stop_flag = false;
            move_flag = false;
            break;
        }

//        if(cnt > 100) { // FIXME 调试用
//            cnt = 0;
//            printf("blocking\n");
//        }

        usleep(1000 * 10);
    }
}

/**
 * 通过读取速度判断是否运动完成，主循环中调用，非阻塞
 */
bool LiftDriver::MoveDone() {
    if(running) {
//        printf("\nrunning...\n");
//        usleep(1000 * 50); // 运动指令发出后，延时100ms再读取速度，等待电机运动
        int16_t speed_ = GetSpeed();
        if(speed_ == 0) {
            running = false;
            printf("\nMove Done!\n");
        }
    }
}


bool LiftDriver::LiftControl(lift_driver::LiftCtl::Request  &req, lift_driver::LiftCtl::Response &res) {
    if(req.command == "Stop") {
        printf("\n[INFO] Lift stop.\n");
        stop_flag = true;
        Stop();
        res.success = true;
    }
    else if(req.command == "Speed") { // 设置速度
        if (req.pose >= 1 && req.pose <= 50) { // 限制升降机构速度范围为1-50 mm/s
            printf("[INFO] Set speed: %.2f mm/s.\n", req.pose);
            SetSpeed(req.pose);
            res.success = true;
        } else {
            printf("[WARN] Lift support speed: 1-100 mm/s.\n");
            res.success = false;
        }
    }
    else if(req.command == "Home") {
        printf("\n[INFO] Lift back home...\n");
        back_home_flag = true;
        MoveDoneBlocking(); // 阻塞直到运动完成
        printf("\n[INFO] Lift back home done.\n");

        res.success = true;
    }
    // 绝对运动的pose单位为mm
    else if(req.command == "Move") {
        if (req.pose >= 0 && req.pose <= 2400) { // 限制升降机构运动范围为0-2.4米
            printf("\n[INFO] Lift MoveABS to pose: %.2f.\n", req.pose);
            move_abs_flag = true;
            move_abs_round = req.pose * 4000; // 1mm-4000脉冲 —— 丝杠1转50mm，电机减速比是20

            MoveDoneBlocking(); // 阻塞直到运动完成
            running = false; // 电机停止
            printf("\n[INFO] MoveABS done.\n");

            res.success = true;
        } else {
            printf("[WARN] Lift support pose: 0-2400 mm.\n");
            res.success = false;
        }
    }
    // 相对运动的pose单位为mm
    else if(req.command == "MoveUp") {
        printf("\n[INFO] MoveUP...\n");
        move_up_flag = true; // 设置标志位
        move_up_round = req.pose *4000; // 设置脉冲数 1mm-4000脉冲 —— 丝杠1转50mm，电机减速比是20

        MoveDoneBlocking(); // 阻塞直到运动完成
        running = false; // 电机停止
        printf("\n[INFO] MoveUP done.\n");

        res.success = true;
    }
    else if(req.command == "MoveDown") {
        printf("\n[INFO] MoveDown...\n");
        move_down_flag = true; // 设置标志位
        move_down_round = req.pose *4000; // 设置脉冲数 1mm-4000脉冲 —— 丝杠1转50mm，电机减速比是20

        MoveDoneBlocking(); // 阻塞直到运动完成
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

bool LiftDriver::LiftStatus(lift_driver::LiftStat::Request  &req, lift_driver::LiftStat::Response &res) {
    if(req.command == "Running") { // 获取电机运行状态
        res.success = running;
    }
    else {
        printf("Supported command: Running\n");
        res.success = false;
    }

    return true;
}

bool LiftDriver::LiftPose(lift_driver::LiftPose::Request  &req, lift_driver::LiftPose::Response &res) {
    if(req.command == "Pose") { // 获取电机运行状态
        res.pose = cur_pose / 4000;
    }
    else {
        printf("Supported command: Pose\n");
    }

    return true;
}

/**
 * ModbusCRC16校验
 */
unsigned short LiftDriver::ModbusCRC16(std::vector<unsigned char> &buff, uint16_t len)
{
    unsigned short tmp = 0xffff;
    unsigned short ret1 = 0;

    for(int n = 0; n < len; n++) { /*要校验的位数为buff.size()个*/
        tmp = buff[n] ^ tmp;
        for(int i = 0; i < 8; i++) {  /*此处的8 -- 指每一个char类型又8bit，每bit都要处理*/
            if(tmp & 0x01) {
                tmp = tmp >> 1;
                tmp = tmp ^ 0xa001;
            }
            else{
                tmp = tmp >> 1;
            }
        }
    }
    /*CRC校验后的值*/
//    printf("%X\n",tmp);
    /*将CRC校验的高低位对换位置*/
    ret1 = tmp >> 8;
    ret1 = ret1 | (tmp << 8);
//    printf("ret: %X\n", ret1);
    return ret1;
}