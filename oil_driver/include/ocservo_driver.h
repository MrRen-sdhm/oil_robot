#include <utility>

//
// Created by sdhm on 9/11/19.
//

#pragma once

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <mutex>
#include <stdexcept>

#include "../3rdparty/serialport/serialport.h"

namespace ocservo {

template<typename T, typename U>
constexpr size_t offsetOf(U T::*member) {
    return (char *) &((T *) nullptr->*member) - (char *) nullptr;
}

struct OCServoRS485Device {

#pragma pack(push)    // 记录当前内存对齐
#pragma pack(1)       // 设定为1字节对齐
    struct Register { // 假定CPU为小端序 73字节
        uint16_t model;
        uint8_t none1;
        uint16_t firmware_version;
        uint8_t id;
        uint8_t baud_rate;
        uint8_t response_delay;
        uint8_t response_level;
        uint16_t min_position;
        uint16_t max_position;
        uint8_t max_temperature;
        uint8_t max_voltage;
        uint8_t min_voltage;
        uint16_t max_load;
        uint8_t pwm_phase_mode;
        uint8_t unload_flag;
        uint8_t led_alarm_flag;
        uint8_t pid_p;
        uint8_t pid_d;
        uint8_t pid_i;
        uint16_t starting_force;
        uint8_t dead_zone_cw;
        uint8_t dead_zone_ccw;
        uint16_t pid_integral_limit;
        uint8_t differential_sampling_coefficient;
        uint8_t load_step; // 每ms, 占空比最多改变若干个单位
        uint8_t position_step;
        uint16_t zero_offset; // 零位校准: 新零位 = 原始零位 + 该值, 顺时针旋转为正方向
        uint8_t run_mode;
        uint8_t angle_feedback_mode;
        uint16_t none2;
        uint8_t none3;
        uint8_t torque_enable;
        uint8_t none4;
        uint16_t goal_position;
        uint16_t goal_time;
        uint16_t goal_velocity;
        uint8_t eeprom_lock;
        uint16_t goal_circle;
        uint8_t relative_move_flag;
        uint32_t none5;
        uint16_t present_position;
        uint16_t present_velocity;
        uint16_t present_load;
        uint8_t present_voltage;
        uint8_t present_temperature;
        uint8_t reg_write_flag;
        uint8_t error;
        uint8_t running_flag;
        uint16_t present_goal_position;
        uint16_t present_current;
        uint16_t present_circle;
    } registers;
#pragma pack(pop) // 还原内存对齐

    static uint32_t offset_goal_position() { return offsetOf(&Register::goal_position); }
    static uint32_t sizeof_goal_position() { return sizeof(Register::goal_position); }

    static uint32_t offset_read_data() { return offsetOf(&Register::present_position); }
    static uint32_t sizeof_read_data() { return 17; }

    uint8_t *memory() {
        return (uint8_t *) &registers;
    }

    // 计算电机当前角度, unit: rad
    float present_position() {
        float pos = registers.present_position / 4096.0f * 2 * (float) M_PI;
        // 转换到[ -pi, pi)
        pos = std::remainder(pos, 2 * (float) M_PI);
        return pos;
    }
    // 计算当前速度, unit: rad/s
    float present_velocity() {
        int vel;
        if (registers.present_velocity >= 32768) {
            vel = (int) 32767 - registers.present_velocity;
        } else {
            vel = registers.present_velocity;
        }
        float RPM = vel / 4096.0f;
        return RPM * 2 * (float) M_PI;
    }

    // 当前电机占空比, range: [-1024, 1023]
    int present_load_raw() {
        int load = registers.present_load;
        if (load >= 1024) {
            load = (int) 1023 - load;
        }
        return load;
    }
    // 当前电机占空比, range: [-1, 1]
    float present_load() {
        int load;
        if (registers.present_load >= 1024) {
            // [1024, 2047] 表示 [1, 1024]
            load = 1023 - (int) registers.present_load;
            return load / -1024.0f;
        } else {
            // [0, 1023] 表示 [-1023 ,0]
            load = registers.present_load;
            return load / -1023.0f;
        }
    }

    // 当前力矩, unit: unknown, 值>=100时可以认为抓到东西了
    int present_effort_raw() {
        int current = registers.present_current;
        if (current >= 32768) {
            current = (int) 32767 - current;
        }
        return current;
    }
    // 当前力矩
    float present_effort() {
        // 相对于最大力矩的百分比
        float effort_percent = present_effort_raw() / 450.0f;
        // 转换到kgcm
        float kgcm = effort_percent * 30.0f;
        // 转换到Nm
        float Nm = kgcm / 100.0f * 9.8f;
        return Nm;
    }

    // 设置电机目标位置, unit: rad, range: unlimited
    int goal_position(float _goal_position) {
        // 若圈数为反向, 则位置应取负值
        if (goal_circle() < 0) {
            _goal_position = -_goal_position;
        }
        // 转换到 [-pi, pi)
        _goal_position = std::remainder(_goal_position, 2 * (float) M_PI);
        // 转换到 [0, 2pi)
        if (_goal_position < 0)
            _goal_position += 2 * (float) M_PI;
        int pos = (uint16_t) (_goal_position / (2 * M_PI) * 4096);
        if (pos >= 4096)
            pos %= 4096;
        else if (pos < 0)
            pos = 0;
        registers.goal_position = (uint16_t) pos;
        return 0;
    }

    // 获取目标位置, unit: rad
    float goal_position() {
        float pos = registers.goal_position / 4096.0f * 2 * float(M_PI);
        // 转换到 [-pi, pi)
        pos = std::remainder(pos, 2 * (float) M_PI);
        // 若圈数为反向, 则位置应取负值
        if (goal_circle() < 0) {
            pos = -pos;
        }
        return pos;
    }

    // 设置最大扭矩, range: [0, 1]
    int max_load(float _max_load) {
        // 限幅到 [0,1]
        _max_load = std::min(1.0f, _max_load);
        _max_load = std::max(0.0f, _max_load);
        // 转换为整型
        int _max_int = int(_max_load * 1024);
        // 限幅到 [0, 1023]
        _max_int = std::min(1023, _max_int);
        _max_int = std::max(0, _max_int);
        if (_max_int != registers.max_load) {
            registers.max_load = _max_int;
            return 0;
        } else {
            // 设定值与给定值相同
            return 1;
        }
    }

    // 获取当前目标位置反馈值, range: [ -pi, pi)
    float present_goal_position() {
        float pos = registers.present_goal_position / 4096.0f * 2 * float(M_PI);
        // 转换到[ -pi, pi)
        pos = std::remainder(pos, 2 * (float) M_PI);
        return pos;
    }

    // 获取当前圈数
    int present_circle() {
        if (registers.present_circle >= 32768) {
            return (int) 32767 - 32768;
        } else {
            return registers.present_circle;
        }
    }
    // 获取当前目标圈数
    int goal_circle() {
        if (registers.goal_circle >= 32768) {
            return (int) 32767 - 32768;
        } else {
            return registers.goal_circle;
        }
    }
};


// 总状态
enum class State {
    INIT,               // 定时1ms
    SYNC_WRITE,         // 写入设定位置
    SYNC_WRITE_SENDING, // 等待数据发送完毕
    REQUEST,            // 请求读取当前位置
    REQUEST_SENDING,    // 等待请求发送完毕
    WAIT_RESPONSE,      // 等待当前位置响应
    PENDING_RESPONSE,   // 接收响应
};

// 数据包解析状态
enum class RxState : uint8_t {
    ID = 0,
    LENGTH = 1,
    DATA = 2,
    WORK_STATE = 3,
    CHECK_SUM = 4,
};

// 寄存器操作指令
enum class OCServoInstructionType : uint8_t {
    NONE = 0x00,
    PING = 0x01,
    READ = 0x02,
    WRITE = 0x03,
    REG_WRITE = 0x04,
    ACTION = 0x05,
    RESET = 0x06,
    SYNC_WRITE = 0x83,
};

// 请求数据包格式
struct Request {
    uint8_t id;
    uint8_t length; // id、指令、数据的字节数之和(数据为 地址+要读取的字节数)
    OCServoInstructionType instruction = OCServoInstructionType::NONE;
    uint8_t *data = NULL;
    uint8_t check_sum;
};

// 响应数据包格式
struct Response {
    uint8_t id;
    uint8_t length; // id、指令、数据的字节数之和(数据为 返回的各寄存器参数)
    uint8_t error;
    uint8_t *data = NULL;
    uint8_t check_sum;

    uint8_t check() {
        uint8_t checksum = 0;
        checksum += id;
        checksum += length;
        checksum += error;
        for (int i = 0; i < length - 2; i++) {
            checksum += data[i];
        }
//            checksum ^= (uint8_t)0x02;
        uint8_t badbit = ~checksum ^check_sum;
        return badbit;
    }
};

// 当前操作参数
struct OCServoOperation {
    uint8_t id = 0;
    OCServoInstructionType instruction = OCServoInstructionType::NONE;
    uint8_t reg_addr = 0;
    uint8_t reg_length = 0;
};

class OCServoRS485{
public:
    OCServoRS485(int id, std::string joint_name) : id_(id), joint_name_(std::move(joint_name)) {
        std::string serialDeviceName_ = "/dev/ttyUSB0";
        serialport_ = new LinuxSerialPort::SerialPort(serialDeviceName_, LinuxSerialPort::BaudRate::B_1000000);
        if (serialport_->Open()) {
            init(); // 舵机初始化
        }
    }

    ~OCServoRS485() {
        serialport_->Close();
    }

    OCServoRS485Device device_ = {};
    int id_ = 1;
    std::string joint_name_;

    bool init_success_ = false;

    LinuxSerialPort::SerialPort* serialport_; // 串口
    unsigned char readData[255] = {0}; // 串口读取缓冲区

    volatile State state_ = State::INIT; // 当前总状态
    volatile RxState rx_state_ = RxState::ID; // 当前接收状态

    OCServoOperation current_op_; // 当前执行的操作
    Request current_req_; // 当前请求帧
    Response current_resp_; // 当前接收的帧

    volatile uint32_t rx_frame_param_count_ = 0; // 接收到的参数个数

    float curr_position_ = 0.0;

    float goal_position_in_r_ = 0.0; // 驱动目标手抓关节位置, 单位为弧度

    float goal_position_in_m_ = 0.0; // ROS中目标手抓关节位置, prismatic类型关节以米为单位
    float curr_position_in_m_ = 0.0; // ROS中当前手抓关节位置, prismatic类型关节以米为单位

    void init();

    // 通信状态机
    void spin_once();

    // 设定指定电机的最大占空比 (异步)
    int set_max_load(int id, float max_load) {
        if (device_.max_load(max_load) == 0) {
            OCServoOperation op;
            op.id = (uint8_t) id;
            op.instruction = OCServoInstructionType::WRITE;
            op.reg_addr = (uint8_t) offsetOf(&OCServoRS485Device::Register::max_load);
            op.reg_length = sizeof(OCServoRS485Device::Register::max_load);
        }
    }

    // 解析一帧消息
    // 返回值: 是完整的一帧=0, 不是一帧=-1
    int _parse_update(uint8_t ch);

    // 转动到指定位置, rad
    bool _goto_position(float rad);

    // 初始化过程中读取数据, 尝试多次
    bool _init_read();

    // 读取并解析消息
    bool _read_parase();

    // 生成读取位置操作
    void _gen_read_operation() {
        // 读当前位置及之后的所有寄存器
        current_op_.id = id_;
        current_op_.instruction = OCServoInstructionType::READ;
        current_op_.reg_addr = OCServoRS485Device::offset_read_data();
        current_op_.reg_length = OCServoRS485Device::sizeof_read_data();
    }

    // 执行一次"请求-响应"操作, 读或写(首地址+长度)
    int _do_operation();
//
//    // 生成合适的同步写操作
//    void _generate_sync_operation() {
//
//        if (!all_motor_disabled) {
//            // 有部分电机被启动
//            for (int id = 1; id <= motor_count_; id++) {
//                devices_[id - 1].registers.torque_enable = motor_enabled_[id - 1];
//            }
//            this->current_sync_op_.ids = motor_enabled_;
//            this->current_sync_op_.reg_addr = DeviceT::offset_goal_position();
//            this->current_sync_op_.reg_length = DeviceT::sizeof_goal_position();
//
//            //            printf("%s[%d-%d]: write position\n", name, current_sync_op.id_start, current_sync_op.id_start + current_sync_op.id_count - 1);
//        } else {
//            // 所有电机被禁用
//            for (int id = 1; id <= motor_count_; id++) {
//                devices_[id - 1].registers.torque_enable = 0;
//                this->current_sync_op_.ids[id - 1] = true;
//            }
//            this->current_sync_op_.reg_addr = (uint16_t) offsetOf(&DeviceT::Register::torque_enable);
//            this->current_sync_op_.reg_length = sizeof(DeviceT::Register::torque_enable);
//
//            //            printf("%s[%d-%d]: write torque_enable\n", name, current_sync_op.id_start, current_sync_op.id_start + current_sync_op.id_count - 1);
//        }
//    }
//
//    // 执行同步写操作
//    int _do_sync_operation() {
//        OCServoSyncOperation<motor_count_> &op = this->current_sync_op_;
//
//        int id_count = 0;
//        for (const bool &enabled : motor_enabled_) {
//            if (enabled) {
//                id_count++;
//            }
//        }
//
//        Request req;
//
//        int drop_count = this->uart_.readsome(NULL, 0xFFFF); // clear rx buffer
//        if (drop_count > 0) {
//            if (!this->ignore_protocol_)
//                printf("%s: %d bytes droped\n", this->name_, drop_count);
//        }
//        rx_state_ = RxState::ID;
//
//        // 帧头
//        while (serialport_->Write((uint8_t *) "\xFF\xFF", 2) != 2);
//
//        // ID
//        req.id = 0xFE;
//        while (serialport_->Write(&(req.id), 1) != 1);
//
//        // 帧长
//        switch (op.instruction) {
//            case OCServoInstructionType::SYNC_WRITE:
//                req.length = uint16_t(4 + id_count * (op.reg_length + 1)); // 发送的数据中包含id
//                break;
//            default:
//                req.length = 0;
//        }
//        while (serialport_->Write((uint8_t *) &(req.length), 1) != 1);
//
//        // 指令
//        req.instruction = op.instruction;
//        while (serialport_->Write((uint8_t *) &(req.instruction), 1) != 1);
//
//        // 帧内容
//        switch (op.instruction) {
//            case OCServoInstructionType::SYNC_WRITE:
//                while (serialport_->Write((uint8_t *) &(op.reg_addr), 1) != 1);
//                while (serialport_->Write((uint8_t *) &(op.reg_length), 1) != 1);
//                for (uint8_t id = 1; id <= motor_count_; id++) {
//                    if (op.ids[id - 1]) {
//                        while (serialport_->Write(&id, 1) != 1);
//                        while (serialport_->Write(&(devices_[id - 1].memory()[op.reg_addr]), op.reg_length) !=
//                               op.reg_length);
//                    }
//                }
//                break;
//            default:
//                break;
//        }
//        req.data = NULL;
//
//        // 校验和
//        req.check_sum = 0;
//        // 各舵机数据和
//        for (int id = 1; id <= motor_count_; id++) {
//            if (op.ids[id - 1]) {
//                req.check_sum += id;
//                for (int i = 0; i < op.reg_length; i++) {
//                    req.check_sum += (&devices_[id - 1].memory()[op.reg_addr])[i];
//                }
//            }
//        }
//        req.check_sum += req.id + req.length + (uint8_t) req.instruction + op.reg_addr + op.reg_length;
//        req.check_sum = ~req.check_sum;
//        while (serialport_->Write(&(req.check_sum), 1, true) != 1);
//
//        this->current_sync_op_ = op;
//        current_req_ = req;
//        current_resp_ = Response();
//        return 0;
//    }

};
}

