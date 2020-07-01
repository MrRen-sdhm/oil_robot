//
// Created by MrRen-sdhm on 20-06-29.
//

#include "lift_driver.h"

#include <cstdlib>
#include <iostream>

LiftDriver::LiftDriver() {
    Init();
}

LiftDriver::LiftDriver(ros::NodeHandle* nh) : nh_(*nh) {
    Init();
}

LiftDriver::~LiftDriver() {
//    serialdriver_->ClosePort();
//    delete serialdriver_;

    ///
    serial_port_->Close();
    delete serial_port_;
}

void LiftDriver::Init() {
    std::string serialDeviceName_ = "/dev/ttyUSB0";

    // 波特率115200, 数据位8位, 偶校验, 一位停止位
//    serialdriver_ = new SerialDriver();
//    bool ret = serialdriver_->OpenPort(serialDeviceName_, 9600, SerialDriver::DataBits::Data8,
//                                       SerialDriver::Parity::Even, SerialDriver::StopBits::Stop1_0);
//    if(!ret) throw std::runtime_error("Open serial of lift failed!\n"); // FIXME

    ///
    try {
        serial_port_ = new SerialPort(serialDeviceName_, BaudRate::BAUD_9600, CharacterSize::CHAR_SIZE_8,
                                      FlowControl::FLOW_CONTROL_NONE, Parity::PARITY_EVEN, StopBits::STOP_BITS_1);
    }
    catch  (const OpenFailed&) {
        throw std::runtime_error("Open serial of lift failed!\n");
    }

    lift_ctl_service = nh_.advertiseService("lift_control", &LiftDriver::LiftControl, this);
    ROS_INFO("Lift control service started!");
}

/**
 * 向上运动， 相对运动（PR1），单位：转
 * 设置指令: 01 10 62 08 00 06 0C 00 41 FF FE 79 60 00 C8 00 32 00 32
 * 00 41 表示相对运动 见PR9.00 / FF FE 79 60 代表-100000转  FF FE为高位 79 60为低位 见PR9.01（高位）及PR9.02（低位）
 * 00 C8 代表速度 / 00 32 代表加速度 / 00 32 代表减速度
 */
void LiftDriver::MoveUp(int32_t round) {
    round = -round; // 向上时脉冲数为负
    std::vector<unsigned char> _Data = {0x01, 0x10, 0x62, 0x08, 0x00, 0x06, 0x0C, 0x00, 0x41};
    // 脉冲数
    _Data.push_back(round >> 24);
    _Data.push_back(round >> 16);
    _Data.push_back(round >> 8 );
    _Data.push_back(round >> 0 );

    // 速度/加速度/减速度
    _Data.push_back(0x00);
    _Data.push_back(0xC8);
    _Data.push_back(0x00);
    _Data.push_back(0x32);
    _Data.push_back(0x00);
    _Data.push_back(0x32);

    // CRC校验
    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    for(auto i : _Data) {
        printf("%.2X ", i);
    }

    serialdriver_->Write(_Data.data(), _Data.size());

    usleep(50000 * 10); // 等待50ms

    // PR1运行
    std::vector<unsigned char> _Data1 = {0x01, 0x06, 0x60, 0x02, 0x00, 0x11};
    ret = ModbusCRC16(_Data1, _Data1.size());
    _Data1.push_back(ret >> 8);
    _Data1.push_back(ret >> 0);

    printf("\n");
    for(auto i : _Data1) {
        printf("%.2X ", i);
    }

    serialdriver_->Write(_Data1.data(), _Data1.size());

    MoveDone(); // 阻塞到移动完成
}

/**
 * 向下运动，相对运动（PR0），单位：转
 * 设置指令: 01 10 62 00 00 06 0C 00 41 00 01 86 A0 00 C8 00 32 00 32
 * 00 41 表示相对运动 见PR9.00 / 00 01 86 A0 代表100000转  00 01为高位 86 A0为低位 见PR9.01（高位）及PR9.02（低位）
 * 00 C8 代表速度 / 00 32 代表加速度 / 00 32 代表减速度
 * 运动指令： 01 06 60 02 00 10
 */
void LiftDriver::MoveDown(int32_t round) {
    std::vector<unsigned char> _Data = {0x01, 0x10, 0x62, 0x00, 0x00, 0x06, 0x0C, 0x00, 0x41};
    // 脉冲数
    _Data.push_back(round >> 24);
    _Data.push_back(round >> 16);
    _Data.push_back(round >> 8 );
    _Data.push_back(round >> 0 );

    // 速度/加速度/减速度
    _Data.push_back(0x00);
    _Data.push_back(0xC8);
    _Data.push_back(0x00);
    _Data.push_back(0x32);
    _Data.push_back(0x00);
    _Data.push_back(0x32);

    // CRC校验
    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    for(auto i : _Data) {
        printf("%.2X ", i);
    }

    serialdriver_->Write(_Data.data(), _Data.size());

    usleep(50000 * 10); // 等待50ms

    // PR0运行
    std::vector<unsigned char> _Data1 = {0x01, 0x06, 0x60, 0x02, 0x00, 0x10};
    ret = ModbusCRC16(_Data1, _Data1.size());
    _Data1.push_back(ret >> 8);
    _Data1.push_back(ret >> 0);

    printf("\n");
    for(auto i : _Data1) {
        printf("%.2X ", i);
    }

    serialdriver_->Write(_Data1.data(), _Data1.size());

    MoveDone(); // 阻塞到移动完成
}

/**
 * 向上运动，绝对运动（PR0），单位：转
 */
void LiftDriver::MoveUpABS(int32_t round) {
    round = -round; // 向上时脉冲数为负
    std::vector<unsigned char> _Data = {0x01, 0x10, 0x62, 0x18, 0x00, 0x06, 0x0C, 0x00, 0x01}; // 第四个字节为0x18 最后一个字节为0x01
    // 脉冲数
    _Data.push_back(round >> 24);
    _Data.push_back(round >> 16);
    _Data.push_back(round >> 8 );
    _Data.push_back(round >> 0 );

    // 速度/加速度/减速度
    _Data.push_back(0x00);
    _Data.push_back(0xC8);
    _Data.push_back(0x00);
    _Data.push_back(0x32);
    _Data.push_back(0x00);
    _Data.push_back(0x32);

    // CRC校验
    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    for(auto i : _Data) {
        printf("%.2X ", i);
    }

    serialdriver_->Write(_Data.data(), _Data.size());

    usleep(50000 * 10); // 等待50ms

    // PR1运行
    std::vector<unsigned char> _Data1 = {0x01, 0x06, 0x60, 0x02, 0x00, 0x13};
    ret = ModbusCRC16(_Data1, _Data1.size());
    _Data1.push_back(ret >> 8);
    _Data1.push_back(ret >> 0);

    printf("\n");
    for(auto i : _Data1) {
        printf("%.2X ", i);
    }

    serialdriver_->Write(_Data1.data(), _Data1.size());

    MoveDone(); // 阻塞到移动完成
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
    _Data.push_back(0x00);
    _Data.push_back(0xC8);
    _Data.push_back(0x00);
    _Data.push_back(0x32);
    _Data.push_back(0x00);
    _Data.push_back(0x32);

    // CRC校验
    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    for(auto i : _Data) {
        printf("%.2X ", i);
    }

    serialdriver_->Write(_Data.data(), _Data.size());

    usleep(50000 * 10); // 等待50ms

    // PR0运行
    std::vector<unsigned char> _Data1 = {0x01, 0x06, 0x60, 0x02, 0x00, 0x12};
    ret = ModbusCRC16(_Data1, _Data1.size());
    _Data1.push_back(ret >> 8);
    _Data1.push_back(ret >> 0);

    printf("\n");
    for(auto i : _Data1) {
        printf("%.2X ", i);
    }

    serialdriver_->Write(_Data1.data(), _Data1.size());

    MoveDone(); // 阻塞到移动完成
}

void LiftDriver::BackHome() {
    // 先切换到回零模式
    std::vector<unsigned char> _Data = {0x01, 0x06, 0x60, 0x0A, 0x00, 0x01};
    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    serialdriver_->Write(_Data.data(), _Data.size());

    usleep(100000 * 10); // 等待50ms

    // 设置回零速度
    std::vector<unsigned char> _Data1 = {0x01, 0x06, 0x60, 0x0F, 0x01, 0x2C};
    ret = ModbusCRC16(_Data1, _Data1.size());
    _Data1.push_back(ret >> 8);
    _Data1.push_back(ret >> 0);

    serialdriver_->Write(_Data1.data(), _Data1.size());

    usleep(100000 * 10); // 等待50ms

    // 回零
    std::vector<unsigned char> _Data2 = {0x01, 0x06, 0x60, 0x02, 0x00, 0x20};
    ret = ModbusCRC16(_Data2, _Data2.size());
    _Data2.push_back(ret >> 8);
    _Data2.push_back(ret >> 0);

    serialdriver_->Write(_Data2.data(), _Data2.size());

    MoveDone(); // 阻塞到移动完成
}

void LiftDriver::Stop() {
    std::vector<unsigned char> _Data = {0x01, 0x06, 0x60, 0x02, 0x00, 0x40};

    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    serialdriver_->Write(_Data.data(), _Data.size());
}

//int32_t LiftDriver::GetPose() {
//    std::vector<unsigned char> _Data = {0x01, 0x03, 0x60, 0x2C, 0x00, 0x02};
//
//    unsigned short ret = ModbusCRC16(_Data, _Data.size());
//    _Data.push_back(ret >> 8);
//    _Data.push_back(ret >> 0);
//
//    serialdriver_->Write(_Data.data(), _Data.size());
//
//    printf("\nTx:");
//    for(auto i : _Data) {
//        printf("%.2X ", i);
//    }
//    printf("\n");
//
//    // 延时读取数据
//    usleep(100000 * 10); // 等待50ms
//
//    // 读取数据
//    int len = serialdriver_->Read(&rx_buf_, 10); // 这里务必读取>9字节
//    printf("len: %d\n", len);
//    if (len == 9) { // 正确接收到9个响应字节
////    if(1) {
//        printf("Rx:");
//        for (int i = 0; i < len; i++)
//            printf("%.2X ", rx_buf_[i]);
//        puts("");
//    }
//
//    int32_t round = 0;
//    round |= (int32_t) rx_buf_[3] << 24;
//    round |= (int32_t) rx_buf_[4] << 16;
//    round |= (int32_t) rx_buf_[5] << 8;
//    round |= (int32_t) rx_buf_[6] << 0;
//
////        int32_t round = ((int32_t)rx_buf_[3] << 24) | ((int32_t)rx_buf_[4] << 16) | ((int32_t)rx_buf_[5] << 8) | (int32_t)rx_buf_[6];
//    printf("\n%d", round);
//    return round;
//}


int32_t LiftDriver::GetPose() {
    std::vector<unsigned char> _Data = {0x01, 0x03, 0x60, 0x2C, 0x00, 0x02};

    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    serial_port_->Write(_Data);

    printf("\nTx:");
    for(auto i : _Data) {
        printf("%.2X ", i);
    }
    printf("\n");

    usleep(1000 * 50); // 延时50ms再读取数据

    DataBuffer read_buffer ; // 接收缓冲区
    size_t ms_timeout = 250 ; // 接收超时，即获取超时时间内的数据

    int32_t round = 0; // 电机当前位置（脉冲数）

    try {
        serial_port_->Read(read_buffer, 0, ms_timeout) ; // 0表示在超时时间内尽可能多的读取数据
    }
    catch (const ReadTimeout&) { // 超时即触发异常，注意此处的超时并不是错误
        int len = read_buffer.size();
        printf("len: %d\n", len);
        if (len == 9) { // 正确接收到9个响应字节
            printf("Rx:");
            for (int i = 0; i < len; i++)
                printf("%.2X ", read_buffer[i]);
            puts("");

            round |= (int32_t) read_buffer[3] << 24;
            round |= (int32_t) read_buffer[4] << 16;
            round |= (int32_t) read_buffer[5] << 8;
            round |= (int32_t) read_buffer[6] << 0;
//            int32_t round = ((int32_t)rx_buf_[3] << 24) | ((int32_t)rx_buf_[4] << 16) | ((int32_t)rx_buf_[5] << 8) | (int32_t)rx_buf_[6];
        }
    }

    return round;
}

int16_t LiftDriver::GetSpeed() {
    std::vector<unsigned char> _Data = {0x01, 0x03, 0x0B, 0x09, 0x00, 0x01};

    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    serialdriver_->Write(_Data.data(), _Data.size());

    printf("\n");
    for(auto i : _Data) {
        printf("%.2X ", i);
    }
    printf("\n");

    // 延时读取数据
    usleep(1000 * 50); // 等待50ms

    int16_t speed = 0;

    // 读取数据
    int len = serialdriver_->Read(&rx_buf_, 10); // 这里务必读取>7字节
//    printf("len: %d\n", len);
    if (len == 7) { // 正确接收到7个响应字节
//        for (int i = 0; i < len; i++)
//            printf("%.2X ", rx_buf_[i]);
//        puts("");

        speed |= (int32_t) rx_buf_[3] << 8;
        speed |= (int32_t) rx_buf_[4] << 0;
        printf("\nSpeed: %d", speed);
        return speed;
    }

    return -1; // 未读取到速度
}

void LiftDriver::GetState() {
    std::vector<unsigned char> _Data = {0x01, 0x03, 0x0B, 0x05, 0x00, 0x01};

    unsigned short ret = ModbusCRC16(_Data, _Data.size());
    _Data.push_back(ret >> 8);
    _Data.push_back(ret >> 0);

    serialdriver_->Write(_Data.data(), _Data.size());

    printf("\n");
    for(auto i : _Data) {
        printf("%.2X ", i);
    }
    printf("\n");

    // 延时读取数据
    usleep(50000 * 10); // 等待50ms

    // 读取数据
    int len = serialdriver_->Read(&rx_buf_, 21); // 这里务必读取>9字节
    printf("len: %d\n", len);
    if (len == 9) { // 正确接收到9个响应字节
//    if(1) {
        for (int i = 0; i < len; i++)
            printf("%.2X ", rx_buf_[i]);
        puts("");
    }

    int32_t speed = 0;
    speed |= (int32_t) rx_buf_[3] << 24;
    speed |= (int32_t) rx_buf_[4] << 16;
    speed |= (int32_t) rx_buf_[5] << 8;
    speed |= (int32_t) rx_buf_[6] << 0;

//        int32_t round = ((int32_t)rx_buf_[3] << 24) | ((int32_t)rx_buf_[4] << 16) | ((int32_t)rx_buf_[5] << 8) | (int32_t)rx_buf_[6];
    printf("\nSpeed: %d", speed);
}

/**
 * 通过读取速度判断是否运动完成，完成前阻塞
 */
bool LiftDriver::MoveDone() {
    usleep(1000 * 100); // 延时100ms再读取速度，等待电机运动
    while(true) {
        int64_t speed = GetSpeed();
        usleep(1000 * 100); // 500ms进行一次位置检测
        if(speed == 0) {
            printf("\nMove Done.\n");
            break;
        }
    }
}

bool LiftDriver::LiftControl(lift_driver::LiftCtl::Request  &req, lift_driver::LiftCtl::Response &res) {
    if(req.command == "Stop") {
        printf("Lift stop.\n");
        Stop();
        res.success = true;
    }
    else if(req.command == "Home") {
        printf("Lift back home.\n");
        BackHome();
        res.success = true;
    }
    // 绝对运动的pose单位为mm
    else if(req.command == "Move") {
        if (req.pose >= 0 && req.pose <= 2400) { // 限制升降机构运动范围为0-2.4米
            printf("Lift move to pose: %d.\n", req.pose);
            MoveABS(req.pose);
            res.success = true;
        } else {
            printf("Lift support pose: 1-1000.\n");
            res.success = false;
        }
    }
    // 相对运动的pose单位为mm
    else if(req.command == "MoveUp") {
        int32_t round = req.pose *4000; // 1mm-4000脉冲 —— 丝杠1转50mm，电机减速比是20
        MoveUp(round);
    }
    else if(req.command == "MoveDown") {
        int32_t round = req.pose *4000; // 1mm-4000脉冲 —— 丝杠1转50mm，电机减速比是20
        MoveUp(round);
    }
    else {
        printf("Supported command: Stop/Move\n");
        res.success = false;
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


// 1mm/s —— 24转/分钟