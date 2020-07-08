//
// Created by MrRen-sdhm on 20-05-30.
//

#include "qr_driver.h"

#include <cstdlib>
#include <iostream>

QRDriver::QRDriver(ros::NodeHandle* nh, string serial_Name) : nh_(*nh) {
    Init(serial_Name);
}

QRDriver::~QRDriver() {
    serial_port->Close();
    delete serial_port;
}

void QRDriver::Init(string serialDeviceName) {
    // 波特率115200, 数据位8位, 偶校验
    try {
        serial_port = new SerialPort(serialDeviceName, BaudRate::BAUD_115200, CharacterSize::CHAR_SIZE_8,
                                     FlowControl::FLOW_CONTROL_NONE, Parity::PARITY_EVEN);
    }
    catch  (const OpenFailed&) {
        throw std::runtime_error("Open serial of lift failed!\n");
    }

//    hand_ctl_service = nh_.advertiseService("hand_control", &QRDriver::HandControl, this);
//    ROS_INFO("Yinshi hand control service started!");
}

/**
 * 设置方向
 * 指令: E4 1B
 */
void QRDriver::SelectDir() {
    std::vector<unsigned char> _Data = {0xE4, 0x1B};
    serial_port->Write(_Data);
}

/**
 * 获取位置, 供主循环调用
 * 指令: C8 37
 * 响应数据长度为21
 */
void QRDriver::GetPose() {
    while(ros::ok()) {
        // 发送指令
        std::vector<unsigned char> _Data = {0xC8, 0x37};
        serial_port->Write(_Data);

        usleep(1000 * 50); // 等待50ms

        try {
            serial_port->Read(read_buffer, 0, ms_timeout) ; // 0表示在超时时间内尽可能多的读取数据
        }
        catch (const ReadTimeout&) { // 超时即触发异常，注意此处的超时并不是错误
            int len = read_buffer.size();
            // printf("len: %d\n", len);

            if(len == 21) { // 正确接收到21个响应字节
//                for (int i = 0; i < 21; i++)
//                    printf("%2X ", read_buffer[i]);
//                puts("");

                Parsing(); // 解析位置
            }
        }
    }
}

void QRDriver::Parsing() {
    if((read_buffer[0]==0)&&(read_buffer[1]==69))//识别到二维码，只计算识别到二维码时的参数，69=0x45
    {
        //二维码参数解析
        int8_t Byte3_1 = read_buffer[2] & 7;
        int8_t X_position = Byte3_1 * 16384 * 128 + read_buffer[3] * 16384 + read_buffer[4] * 128 + read_buffer[5]; // X轴偏差
        int8_t Y_position = read_buffer[6] * 128 + read_buffer[7]; // Y轴偏差
        uint16_t Angle_value = read_buffer[10] * 128 + read_buffer[11]; // 旋转角度,无符号16位数
        uint8_t Tag_number = read_buffer[17]; //标签号，无符号8位数

        printf("X:%d Y:%d Angle:%d Tag:%d\n", X_position, Y_position, Angle_value, Tag_number);

        /** 发布消息 **/
        std_msgs::Int16MultiArray msg;
        msg.data.push_back(X_position);
        msg.data.push_back(Y_position);
        msg.data.push_back(Angle_value);
        msg.data.push_back(Tag_number);
        pub.publish(msg);
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
