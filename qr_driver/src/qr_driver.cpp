//
// Created by MrRen-sdhm on 20-05-30.
//

#include "qr_driver.h"

#include <cstdlib>
#include <iostream>

QRDriver::QRDriver() {
    Init();
}

QRDriver::QRDriver(ros::NodeHandle* nh) : nh_(*nh) {
    Init();
}

QRDriver::~QRDriver() {
    serialdriver_->ClosePort();
    delete serialdriver_;
}

void QRDriver::Init() {
    std::string serialDeviceName_ = "/dev/ttyUSB0";

    // 波特率115200, 数据位8位, 偶校验
    serialdriver_ = new SerialDriver();
    bool ret = serialdriver_->OpenPort(serialDeviceName_, 115200, SerialDriver::DataBits::Data8, SerialDriver::Parity::Even);
    if(!ret) throw std::runtime_error("Open serial of qr failed!\n"); // FIXME

//    hand_ctl_service = nh_.advertiseService("hand_control", &QRDriver::HandControl, this);
//    ROS_INFO("Yinshi hand control service started!");
}

/**
 * 设置方向
 * 指令: E4 1B
 */
void QRDriver::SelectDir() {
    std::vector<unsigned char> _Data = {0xE4, 0x1B};
    serialdriver_->Write(_Data.data(), _Data.size());
}

/**
 * 获取位置, 供主循环调用
 * 指令: C8 37
 * 响应数据长度为21
 */
void QRDriver::GetPose() {
//    // 发送指令
//    std::vector<unsigned char> _Data = {0xC8, 0x37};
//    serialdriver_->Write(_Data.data(), _Data.size());

    while(true) {
        // 发送指令
        std::vector<unsigned char> _Data = {0xC8, 0x37};
        serialdriver_->Write(_Data.data(), _Data.size());

        usleep(50000 * 10); // 等待50ms

        // 读取数据
        int len = serialdriver_->Read(&rx_buf_, 22); // 这里务必读取>21字节
        // printf("len: %d\n", len);
        if(len == 21) { // 正确接收到21个响应字节
            for (int i = 0; i < 21; i++)
                printf("%2X ", rx_buf_[i]);
            puts("");

            Parase(); // 解析位置
        }
    }
}

void QRDriver::Parase() {
    if((rx_buf_[0]==0)&&(rx_buf_[1]==69))//识别到二维码，只计算识别到二维码时的参数，69=0x45
    {
        //二维码参数解析
        int8_t Byte3_1 = rx_buf_[2] & 7;
        int8_t X_position = Byte3_1 * 16384 * 128 + rx_buf_[3] * 16384 + rx_buf_[4] * 128 + rx_buf_[5]; // X轴偏差
        int8_t Y_position = rx_buf_[6] * 128 + rx_buf_[7]; // Y轴偏差
        uint16_t Angle_value = rx_buf_[10] * 128 + rx_buf_[11]; // 旋转角度,无符号16位数
        uint8_t Tag_number = rx_buf_[17]; //标签号，无符号8位数

        printf("X:%d Y:%d Angle:%d Tag:%d\n", X_position, Y_position, Angle_value, Tag_number);
    }
}

//bool QRDriver::HandControl(yinshi_driver::HandControl::Request  &req, yinshi_driver::HandControl::Response &res) {
//    if(req.command == "Open") {
//        printf("Yinshi hand open.\n");
//        Move(QRDriver::HandCMD::OpenHand, 1000, 1000);
//        res.success = true;
//    }
//    else if(req.command == "Close") {
//        printf("Yinshi hand close.\n");
//        Move(QRDriver::HandCMD::CloseHand, 1000, 1000);
//        res.success = true;
//    }
//    else if(req.command == "Move") {
//        if (req.pose > 0 && req.pose <=1000) {
//            printf("Yinshi hand move to pose: %d.\n", req.pose);
//            MovePose(req.pose);
//            res.success = true;
//        } else {
//            printf("Yinshi hand support pose: 1-1000.\n");
//            res.success = false;
//        }
//    }
//    else {
//        printf("Supported command: Open/Close\n");
//        res.success = false;
//    }
//
//    return true;
//}
