//
// Created by sdhm on 7/5/19.
//

#pragma once

#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <utility>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <array>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

#include "modbusadapter.h"

#define R2D(rad) ((double)(rad) / (double)M_PI * 180.0f)
#define D2R(deg) ((double)(deg) * (double)M_PI / 180.0f)

using namespace std::chrono;

// 获取寄存器偏移量
template<typename T, typename U>
constexpr size_t offsetOf(U T::*member) {
    return (int16_t *) &((T *) nullptr->*member) - (int16_t *) nullptr; }

struct OilDevice {
    static const int cycle_step = 262144; // 编码器一圈分辨率
    array<uint8_t, 7> reduction_ratio = {{120, 120, 120, 100, 100, 100, 100}}; // 齿轮减速比

#pragma pack(push) // 记录当前内存对齐
#pragma pack(1)    // 设定为1字节对齐, 高地址对应高位, 如4011对应低八位, 4012对应高八位
    // 内存区间总长度为 (220字节, 110个数据, 存储于110个寄存器) uint16_t -> 2字节 -> 16位 -> 1个数据
    struct Register {
        /***  写  ***/
        int32_t target_position_1; // 40011-40012 4字节 2个数据 %MW10-%MW11
        int32_t target_position_2; // 40013-40014 4字节 2个数据 %MW12-%MW13
        int32_t target_position_3;
        int32_t target_position_4;
        int32_t target_position_5;
        int32_t target_position_6;
        int32_t target_position_7;

        int16_t target_velocity_1;
        int16_t target_velocity_2;
        int16_t target_velocity_3;
        int16_t target_velocity_4;
        int16_t target_velocity_5;
        int16_t target_velocity_6;
        int16_t target_velocity_7;

        int16_t target_torque_1;
        int16_t target_torque_2;
        int16_t target_torque_3;
        int16_t target_torque_4;
        int16_t target_torque_5;
        int16_t target_torque_6;
        int16_t target_torque_7;  // %MW37

        /***  读  ***/
        // 8字节
        int32_t position_value_1; // 40039-40040 4字节 2个数据 %MW38-%MW39
        int16_t velocity_value_1; // 40041       2字节 1个数据 %MW40
        int16_t torque_value_1;   // 40042       2字节 1个数据 %MW41
        // 8字节
        int32_t position_value_2; // 40043-40044
        int16_t velocity_value_2;
        int16_t torque_value_2;
        // 8字节
        int32_t position_value_3;
        int16_t velocity_value_3;
        int16_t torque_value_3;
        // 8字节
        int32_t position_value_4;
        int16_t velocity_value_4;
        int16_t torque_value_4;
        // 8字节
        int32_t position_value_5;
        int16_t velocity_value_5;
        int16_t torque_value_5;
        // 8字节
        int32_t position_value_6;
        int16_t velocity_value_6;
        int16_t torque_value_6;
        // 8字节
        int32_t position_value_7;
        int16_t velocity_value_7;
        int16_t torque_value_7;   // %MW65

        // 28字节
        int32_t zero_position_1; // 40067-40068 4字节 2个数据 %MW66-%MW67
        int32_t zero_position_2;
        int32_t zero_position_3;
        int32_t zero_position_4;
        int32_t zero_position_5;
        int32_t zero_position_6;
        int32_t zero_position_7; // %MW78-%MW79

        // 14字节
        int16_t err_code_1; // %MW80
        int16_t err_code_2;
        int16_t err_code_3;
        int16_t err_code_4;
        int16_t err_code_5;
        int16_t err_code_6;
        int16_t err_code_7; // %MW86

        // 2字节
        int16_t ctrller_status; // 控制器状态  %MW87

    } registers;

    struct BitRegister {
        // 16字节 8个数据
        int16_t jog; // 40011-40012 offset 0
        int16_t reset_power;
        int16_t back_home;

//        int8_t reset;
//        int8_t power;
//        int8_t back;
//        int8_t home;
    } bit_registers;
#pragma pack(pop) // 还原内存对齐

    static uint8_t offset_goal_position(int id) {
//        printf("[DEBUG] offsetOf target_position_1: %zu\n", offsetOf(&Register::target_position_1));
        return offsetOf(&Register::target_position_1) + 2*(id-1); // 各目标位置寄存器相差2字
    }
    static uint8_t offset_curr_position(int id) {
//        printf("[DEBUG] offsetOf position_value_1: %zu\n", offsetOf<>(&Register::position_value_1));
        return offsetOf(&Register::position_value_1) + 4*(id-1); // 各当前位置寄存器相差4字
    }
    static uint8_t offset_curr_velocity(int id) { return offsetOf(&Register::velocity_value_1) + 4*(id-1); }
    static uint8_t offset_curr_effort(int id) { return offsetOf(&Register::torque_value_1) + 4*(id-1); }

    int16_t *memory() {
        return (int16_t *) &registers;
    }

    int16_t *write_memory() {
        return (int16_t *) &registers.target_position_1;
    }

    int16_t *read_memory() {
        return (int16_t *) &registers.position_value_1;
    }

    int16_t *bit_memory() {
        return (int16_t *) &bit_registers;
    }

    // 返回目标位置指针
    int32_t *goal_pos_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto target_position_1_ptr = (int16_t *) &registers.target_position_1;
            return (int32_t *)(target_position_1_ptr + 2*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("\033[1;31mGiven a bad id when get goal position!\033[0m\n");
        }
    }

    // 返回当前位置指针
    int32_t *curr_pos_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto position_1_ptr = (int16_t *)&registers.position_value_1;
            return (int32_t *)(position_1_ptr + 4*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("\033[1;31mGiven a bad id when get current position!\033[0m\n");
        }
    }

    // 返回当前速度指针
    int16_t *curr_vel_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto position_1_ptr = (int16_t *)&registers.velocity_value_1;
            return (int16_t *)(position_1_ptr + 4*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("\033[1;31mGiven a bad id when get goal velocity!\033[0m\n");
        }
    }

    // 返回当前扭矩指针
    int16_t *curr_eff_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto position_1_ptr = (int16_t *)&registers.torque_value_1;
            return (int16_t *)(position_1_ptr + 4*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("\033[1;31mGiven a bad id when get goal velocity!\033[0m\n");
        }
    }

    // 返回零点位置指针
    int32_t *zero_pos_ptr(uint8_t id) {
        if (1 <= id && id <= 7) {
            auto zero_position_1_ptr = (int16_t *)&registers.zero_position_1;
            return (int32_t *)(zero_position_1_ptr + 2*(id-1)); // 强制转换为uint32_t指针
        } else {
            throw runtime_error("\033[1;31mGiven a bad id when get zero position!\033[0m\n");
        }
    }

    // 设定虚拟寄存器中 目标关节角度 单位为度的10000倍
    bool set_goal_position(uint8_t id, double rad) {
        if (!std::isfinite(rad)) {
            return false;
        }

        double deg = R2D(rad);
        auto pos = int32_t (10000.0*deg);
        *goal_pos_ptr(id) = pos; // 写位置

        return true;
    }

    // 设定虚拟寄存器中 当前关节角度 FIXME:仅用于调试
    bool set_curr_position(uint8_t id, double deg) {
        auto pos = uint32_t (deg * 10000);
        *curr_pos_ptr(id) = pos; // 写位置
        return true;
    }

    // 获取目标位置
    double get_goal_position(uint8_t id) {
        auto pos_raw = int32_t (*goal_pos_ptr(id)); // 原始位置数据, 单位为度, 并放大10000倍
        double pos = D2R((double)pos_raw / 10000.0f); // 缩小一万倍并转换为弧度
        return pos;
    }
    // 获取当前位置(原始值)
    double get_curr_position(uint8_t id) {
        auto pos_raw = int32_t (*curr_pos_ptr(id)); // 原始位置数据, 单位为度, 并放大10000倍
        double pos = D2R((double)pos_raw / 10000.0f); // 缩小一万倍并转换为弧度
        return pos;
    }
    // 获取零点位置
    double get_zero_position(uint8_t id) {
        auto zero_pos_raw = int32_t (*zero_pos_ptr(id)); // 原始位置数据, 单位为度, 并放大10000倍
        double zero_pos = D2R((double)zero_pos_raw / 10000.0f); // 缩小一万倍并转换为弧度
        return zero_pos;
    }
    // 获取当前速度
    double get_curr_velocity(uint8_t id) {
        auto vel_raw = int32_t (*curr_vel_ptr(id)); // 原始速度数据, 单位为转每分
        double vel = (double)vel_raw * (double)M_PI/30.0f; // 1转每分 = 360/60度每秒 = 2pi/60弧度每秒 = pi/30弧度每秒
        return vel;
    }
    // 获取当前力矩
    double get_curr_effort(uint8_t id) {
        const uint16_t rated_moment = 1; // 额定力矩 FIXME：额定力矩大小
        auto eff_raw = int32_t (*curr_eff_ptr(id)); // 原始力矩数据, 为额定力矩的千分比
        double eff = (double)eff_raw/1000.0f * rated_moment;
        return eff;
    }
};

namespace Oil {
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::time_point<Clock> TimePoint;

class MotorDriver {
public:
    static const uint8_t motor_cnt_ = 7; // 电机数
    std::vector<std::string> joint_names_{}; // 关节名称
    bool power_on_flag_ = false; // 电机使能标志
    bool init_ready_flag_ = false; // 控制器初始化完成标志

    /// HMI控制标志
    bool set_home_ctrl_flag_ = false;  // 置零控制标志
    bool reset_ctrl_flag_ = false;     // 复位控制标志
    bool power_ctrl_flag_ = false;     // 使能控制标志
    bool emergency_stop_flag_ = false; // 急停标志

    /// 关节位置相关参数
    std::array<double, motor_cnt_> zero_pos{}; // 读取零位数据的缓冲区
    std::array<double, motor_cnt_> curr_pos{}; // 读取位置数据的缓冲区
    std::array<double, motor_cnt_> curr_vel{}; // 读取速度数据的缓冲区
    std::array<double, motor_cnt_> curr_eff{}; // 读取力矩数据的缓冲区

    MotorDriver(string ip, int port, int slaver, const string& joint_prefix, bool show_info); // 构造函数

    /// 外部接口
    // 设置虚拟寄存器中目标关节位置,加零位
    bool set_position(uint8_t id, double rad) {
        rad = rad + zero_pos[id-1]; /// 在零点位置的基础上生成目标位置
        return device_.set_goal_position(id, rad);
    }

    // 获取虚拟寄存器中当前关节位置,减零位
    double get_position(uint8_t id) {
        return device_.get_curr_position(id) - zero_pos[id-1];
    }
    // 获取虚拟寄存器中目标关节位置,减零位 FIXME:调试用
    double get_goal_position(uint8_t id) {
        return device_.get_goal_position(id) - zero_pos[id-1];
    }
    bool do_write_operation(); // 写控制器寄存器
    bool do_read_operation();  // 读控制器寄存器

    void print_position(uint8_t time);

private:
    /// motor_driver参数
    double last_print_time_ = 0; // 上次状态打印时间
    /// modbus相关参数
    ModbusAdapter *m_master_;        // modbus服务器
    int slaver_;                     // modbus客户端id
    ros::NodeHandle nh_;             // ROS节点句柄
    ros::Subscriber sub_hmi_;        // 上位机消息订阅
    bool do_read_flag_  = false;     // 读取标志
    bool do_write_flag_ = false;     // 写入标志
    bool show_info_ = false; // 打印输出标志

    /// 寄存器相关参数
    OilDevice device_; // 机械臂
    const uint32_t hmi_addr_head_ = 10;
    const uint32_t hmi_addr_write_head_  = 10;  // HMI字节写操作首地址 HMI:40011->modbus addr %MW10 即第10个字开始
    const uint32_t hmi_addr_read_head_  = 38;   // HMI字节读操作首地址 HMI:40039->modbus addr %MW38 即第38个字开始
    const uint32_t hmi_bit_addr_head_ = 0; // HMI位操作首地址   HMI:40000->modbus addr %MW0 即第0个字开始
    const uint32_t hmi_addr_power_ = 01; // 高八位为关节使能地址, 10-16位控制各关节使能, 9位控制所有关节
    const uint32_t hmi_addr_reset_ = 01; // 低八位为关节使能地址, 1-8位控制各关节复位, 1位控制所有关节
    const uint32_t hmi_addr_home_  = 02; // 高八位为关节置零地址, 10-16位控制各关节置零, 9位控制所有关节
    const uint32_t hmi_addr_back_  = 02; // 低八位为关节回零地址, 1-8位控制各关节回零, 1位控制所有关节
    size_t reg_size_ = sizeof(typename OilDevice::Register); // OilDevice::Register尺寸
    size_t ctrller_reg_len_ = reg_size_/2; // 控制器寄存器数量, 即uint16_t数据量
    size_t write_reg_len_ = 28; // 可写寄存器数量, 4*7 = 28
    size_t read_reg_len_ = ctrller_reg_len_ - write_reg_len_; // 可读寄存器数量

    size_t bit_reg_size_ = sizeof(typename OilDevice::BitRegister); // OilDevice::BitRegister尺寸
    size_t bit_reg_len_ = bit_reg_size_/2; // 控制器寄存器数量, 即uint16_t数据量

    // modbus读保持寄存器数据, 使用uint16_t类型写入虚拟寄存器, 使用数据时将虚拟寄存器数据转换为int16或int32类型即可
    int _read_data(int addr, int len, uint16_t* data) {
        int rect = m_master_->modbusReadHoldReg(slaver_, addr, len, data);
        return rect;
    }

    // modbus写保持寄存器数据, 使用uint16_t类型写入, int16或int32类型数据需转换为uint16类型
    int _write_data(int addr, int len, const uint16_t* data) {
        return m_master_->modbusWriteData(slaver_, MODBUS_FC_WRITE_MULTIPLE_REGISTERS, addr, len, data);
    }

    // 使能所有关节
    int enable_all_power() {
        int16_t enable_all = 0x00FF; // 低8位位置1
        return _write_data(hmi_addr_power_, 1, (uint16_t*) &enable_all);
    }

    // 去使能所有关节
    int disable_all_power() {
        int16_t disable_all = 0x0000; // 低8位置0
        return _write_data(hmi_addr_power_, 1, (uint16_t*) &disable_all);
    }

    // 复位所有关节
    int reset_all_joints() {
        int16_t reset_all = 0xFF00; // 高8位置1

        for (int id = 1; id <= motor_cnt_; id++) { // 确保复位后目标位置与当前位置相同 NOTE:必须放在写目标关节位置寄存器之前
            set_position(id, curr_pos[id-1]);
        }

        return _write_data(hmi_addr_reset_, 1, (uint16_t*) &reset_all);
    }

    // 设置为零位
    int set_all_home() {
        int16_t set_all = 0x00FF; // 低8位置1
//        std::fill(curr_pos.begin(), curr_pos.end(), 0); // 当前位置缓冲区置零
        int ret =  _write_data(hmi_addr_home_, 1, (uint16_t*) &set_all);
        return ret;
    }

    void hmi_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
};

}

