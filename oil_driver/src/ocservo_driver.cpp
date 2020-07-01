//
// Created by sdhm on 9/11/19.
//

#include "ocservo_driver.h"

namespace ocservo {

    void OCServoRS485::init() {
        OCServoOperation op;

        // 初始化指令
        op.id = id_;

        /// ************************ 初始化操作-写 ************************ ///

        // 禁用舵机输出
        device_.registers.torque_enable = 1;
        op.instruction = OCServoInstructionType::WRITE;
        op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::torque_enable);
        op.reg_length = sizeof(OCServoRS485Device::Register::torque_enable);
        current_op_ = op; // 获取操作指令
        _do_operation(); // 执行操作

        // 写入零位
        device_.registers.zero_offset = 1798; // 零位
        op.instruction = OCServoInstructionType::WRITE;
        op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::zero_offset);
        op.reg_length = sizeof(OCServoRS485Device::Register::zero_offset);
        current_op_ = op; // 获取操作指令
        _do_operation(); // 执行操作

        // 设定并写入占空比步长
        device_.registers.load_step = 255;
        op.instruction = OCServoInstructionType::WRITE;
        op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::load_step);
        op.reg_length = sizeof(OCServoRS485Device::Register::load_step);
        current_op_ = op; // 获取操作指令
        _do_operation(); // 执行操作

        // 设定并写入默认最大占空比
        device_.max_load(0.4);
        op.instruction = OCServoInstructionType::WRITE;
        op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::max_load);
        op.reg_length = sizeof(OCServoRS485Device::Register::max_load);
        current_op_ = op; // 获取操作指令
        _do_operation(); // 执行操作

        // 设置初始位置 闭合:0.64 张开: 0.00
        device_.goal_position(0.0);
        op.instruction = OCServoInstructionType::WRITE;
        op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::goal_position);
        op.reg_length = sizeof(OCServoRS485Device::Register::goal_position);
        current_op_ = op; // 获取操作指令
        _do_operation(); // 执行操作

        sleep(1); // 读取所有寄存器前须更长时间间隔

        /// ************************ 初始化操作-读 ************************ ///

        // 读取所有寄存器, 串口无法一次读取, 分两次读取
        op.instruction = OCServoInstructionType::READ;
        op.reg_addr = offsetOf(&OCServoRS485Device::Register::id);
        op.reg_length = 33; // 最多只能一次读33字节
        current_op_ = op; // 获取操作指令
        _do_operation(); // 执行操作
        if(!_init_read()) // 初始化读取并解析
            throw std::runtime_error("\033[1;31mOcServo init failed, please check the power of the servo, or restart the program!\033[0m\n");

        op.instruction = OCServoInstructionType::READ;
        op.reg_addr = offsetOf(&OCServoRS485Device::Register::torque_enable);
        op.reg_length = 33; // 最多只能一次读33字节
        current_op_ = op; // 获取操作指令
        _do_operation(); // 执行操作
        if(!_init_read()) // 初始化读取并解析
            throw std::runtime_error("\033[1;31mOcServo init failed, please check the power of the servo, or restart the program!\033[0m\n");

        init_success_ = true;
        printf("[INFO] OcServo init success!\n");

//        printf("zero_offset: %d\n", device_.registers.zero_offset);
//        printf("present_position: %f\n", device_.present_position());
//        printf("present_goal_position: %f\n", device_.present_goal_position());
    }

    bool OCServoRS485::_goto_position(float rad) {
        OCServoOperation op;

        // 初始化指令
        op.id = id_;

        device_.goal_position(rad);
        op.instruction = OCServoInstructionType::WRITE;
        op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::goal_position);
        op.reg_length = sizeof(OCServoRS485Device::Register::goal_position);
        current_op_ = op; // 获取操作指令
        if (_do_operation() == 0) {
            curr_position_  = rad;
            return true;
        } else {
            return false;
            curr_position_  = 0.0;
        }
    }

    bool OCServoRS485::_init_read() {
        int loop_cnt = 0;
        while(true) { // 接收返回的数据
            loop_cnt++; // 循环计数
            bool ret = _read_parase();

            if (ret) { // 得到完整的数据包
                return true;
            } else if (loop_cnt > 5) { // 超时
                return false;
            }
        }
    }

    bool OCServoRS485::_read_parase() {
        int packet_len = 0;
        int ret = -1;
        int loop_cnt = 0;
        while(true) { // 接收返回的数据
            loop_cnt++; // 循环计数
            int len = serialport_->Read(readData);
            if (len > 0) {
                packet_len += len;
                for (int i = 0; i < len; i++) {
//                    printf("%02X\n", readData[i]);
                    ret = _parse_update(readData[i]);
                }
            }

            if (ret == 0) { // 得到完整的数据包
                printf("[INFO] Get a complete packet.\n");
                printf("[INFO] Packet length = %d\n", packet_len);
                printf("[INFO] loop_cnt: %d\n", loop_cnt);
                return true;
            } else if (loop_cnt > 2000) { // 超时
                printf("[WARN] _read_parase time out, loop_cnt: %d\n", loop_cnt);
                return false;
            }
        }
    }

    void OCServoRS485::spin_once() {
        switch (state_) {
            case State::SYNC_WRITE:
                // 准备发送数据 同步写各关节目标位置
//                _generate_sync_operation();
//                _do_sync_operation();

                state_ = State::REQUEST;
                break;
            case State::REQUEST:
                // 准备发送读请求 读取当前位置之后所有寄存器
//                _gen_read_operation(); // 生成操作指令
//                _do_operation(); // 执行操作
                state_ = State::PENDING_RESPONSE;
                break;
            case State::PENDING_RESPONSE:
                // 接收完整的响应帧 (读取当前位置)
                _read_parase();
                state_ = State::SYNC_WRITE;
                break;
        }
    }

    int OCServoRS485::_do_operation() {
        usleep(300); // 受串口速率限制, 各操作须间隔一定时间！！！

        OCServoOperation &op = current_op_;
        Request req;
        Response resp;

        rx_state_ = RxState::ID;

        // 帧头
        serialport_->Write((uint8_t *) "\xFF\xFF", 2);

        // ID
        req.id = op.id;
        resp.id = op.id;
        serialport_->Write(&(req.id), 1);

        // 帧长
        switch (op.instruction) {
            case OCServoInstructionType::PING:
                req.length = 2;
                resp.length = 0;
                break;
            case OCServoInstructionType::READ:
                req.length = 4;
                resp.length = op.reg_length;
                break;
            case OCServoInstructionType::WRITE:
                req.length = op.reg_length + (uint8_t) 3;
                resp.length = 0;
                break;
            case OCServoInstructionType::REG_WRITE:
                req.length = op.reg_length + (uint8_t) 2;
                resp.length = 0;
                break;
            case OCServoInstructionType::ACTION:
                req.length = 2;
                resp.length = 0;
                break;
            case OCServoInstructionType::RESET:
                req.length = 2;
                resp.length = 0;
                break;
            default:
                req.length = 0;
                resp.length = 0;
        }
        resp.length += 2;
        serialport_->Write((uint8_t *) &(req.length), 1);

        // 指令
        req.instruction = op.instruction;
        serialport_->Write((uint8_t *) &(req.instruction), 1);

        // 帧内容
        switch (op.instruction) {
            case OCServoInstructionType::PING:
                break;
            case OCServoInstructionType::READ:
                serialport_->Write((uint8_t *) &(op.reg_addr), 1);
                serialport_->Write((uint8_t *) &(op.reg_length), 1);
                break;
            case OCServoInstructionType::WRITE:
                serialport_->Write((uint8_t *) &(op.reg_addr), 1);
                serialport_->Write(&(device_.memory()[op.reg_addr]), op.reg_length);
                break;
            case OCServoInstructionType::REG_WRITE:
                serialport_->Write((uint8_t *) &(op.reg_addr), 1);
                serialport_->Write(&(device_.memory()[op.reg_addr]), op.reg_length);
                break;
            case OCServoInstructionType::ACTION:
                break;
            case OCServoInstructionType::RESET:
                break;
            default:
                break;
        }
        req.data = NULL;
        resp.data = NULL;

        // 校验和
        req.check_sum = 0;
        switch (op.instruction) {
            case OCServoInstructionType::WRITE:
            case OCServoInstructionType::REG_WRITE:
                req.check_sum += req.id + req.length + (uint8_t) req.instruction + op.reg_addr;
                for (int i = 0; i < op.reg_length; i++) {
                    req.check_sum += device_.memory()[op.reg_addr + i];
                }
                break;
            case OCServoInstructionType::READ:
                req.check_sum += req.id + req.length + (uint8_t) req.instruction + op.reg_addr + op.reg_length;
                break;
            default:
                break;
        }
        req.check_sum = ~req.check_sum;
        serialport_->Write(&(req.check_sum), 1);

        this->current_op_ = op;
        current_req_ = req;
        current_resp_ = resp;

        return 0;
    }


    int OCServoRS485::_parse_update(uint8_t ch) {
        //        printf("[%02X]", ch);
        Response &resp = current_resp_;
        switch (rx_state_) {
            case RxState::ID:
                if (ch == 0xFF) {
                    // 跳过 0xFF
                } else if (ch != resp.id) {
                    rx_state_ = RxState::ID;
                } else {
                    resp.id = ch;
                    rx_state_ = RxState::LENGTH;
                }
                break;
            case RxState::LENGTH:
                if (ch != resp.length) {
                    rx_state_ = RxState::ID;
                } else {
                    rx_state_ = RxState::WORK_STATE;
                }
                break;
            case RxState::WORK_STATE:
                resp.error = ch;
                if (resp.length > 2) {
                    rx_state_ = RxState::DATA;
                    resp.data = device_.memory() + current_op_.reg_addr;
                    rx_frame_param_count_ = 0;
                } else {
                    rx_state_ = RxState::CHECK_SUM;
                }
                break;
            case RxState::DATA:
                if (current_op_.instruction == OCServoInstructionType::READ) {
                    resp.data[rx_frame_param_count_] = ch;
                }
                rx_frame_param_count_++;
                if (rx_frame_param_count_ >= resp.length - 2) { // 数据长度为包有效数据长度减2
                    rx_frame_param_count_ = 0;
                    rx_state_ = RxState::CHECK_SUM;
                }
                break;
            case RxState::CHECK_SUM:
                resp.check_sum = ch;
                rx_state_ = RxState::ID;
                uint8_t check = resp.check();
                if (check == 0) { // 是完整的一帧
                    return 0;
                }
        }
        return 1;
    }
}

